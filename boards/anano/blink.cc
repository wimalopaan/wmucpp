/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// sudo avrdude -p attiny85 -P usb -c avrisp2 -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m
// avrdude -p m328p -c arduino -b 57600 -P /dev/ttyUSB0 -U flash:w:blink.elf

#define NDEBUG

#include <stdlib.h>

#include "../include/anano.h"

#include "hal/constantrate.h"
#include "hal/event.h"
#include "hal/alarmtimer.h"
#include "mcu/avr/usart.h"

#include "console.h"

using systemTimer = AVR::Timer8Bit<0>;

using alarmTimer  = AlarmTimer<systemTimer>;

const auto secondsTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);

using terminalDevice = AVR::Usart<0>;
using terminal = std::basic_ostream<terminalDevice>;

using systemConstantRate = ConstantRateAdapter<1, void, AVR::ISR::Timer<0>::CompareA, alarmTimer>;

using isrRegistrar = IsrRegistrar<systemConstantRate, terminalDevice::RxHandler, terminalDevice::TxHandler>;

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(std::byte b) {
        auto timer = std::to_integer<uint7_t>(b);
        if (timer == *secondsTimer) {
            led::toggle();
            std::outl<terminal>("Nano"_pgm);
        }
        return true;
    }
};

using allEventHandler = EventHandlerGroup<TimerHandler>;

int main() {
    isrRegistrar::init();
    terminalDevice::init<9600>();
    alarmTimer::init();    

    led::dir<AVR::Output>();
    led::on();    
    {
        Scoped<EnableInterrupt<>> ei;
        EventManager::run3<allEventHandler>([](){
            systemConstantRate::periodic();
        });
    }
}

ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}

ISR(USART_UDRE_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}

ISR(USART_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
