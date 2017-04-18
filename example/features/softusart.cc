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

#include <stdlib.h>

#include "mcu/avr8.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/swusart.h"
#include "hal/event.h"
#include "mcu/ports.h"
#include "hal/alarmtimer.h"
#include "std/literals.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/spi.h"
#include "external/ws2812.h"
#include "mcu/avr/delay.h"
#include "console.h"

using spiInput = AVR::Spi<0>;
using terminal = SWUsart<0>;

using systemClock = AVR::Timer8Bit<0>;
using systemTimer = AlarmTimer<systemClock>;

using sampler = PeriodicGroup<AVR::ISR::Timer<0>::CompareA, systemTimer>;

using isrReg = IsrRegistrar<sampler, spiInput, terminal::ReceiveBitHandler, terminal::TransmitBitHandler, terminal::StartBitHandler>; 

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

struct  Spi0handler: public EventHandler<EventType::Spi0> {
    static bool process(const uint8_t& v) {
        Util::put<terminal, true>((char)v);
        return true;
    }
};

struct Timerhandler: public EventHandler<EventType::Timer> {
    static bool process(const uint8_t&) {
        if (spiInput::leak()) {
        }
        else {
        }
        return true;
    }
};


int main()
{
    isrReg::init();
    systemTimer::init();

    terminal::init<2400>();

    std::cout << "Spi Usart Bridge Test 0.1" << std::endl;

    systemTimer::create(1_s, AlarmFlags::Periodic);

    spiInput::init<AVR::SpiSlave<>>();

    using handler = EventHandlerGroup<Spi0handler, Timerhandler>;

    {
        Scoped<EnableInterrupt> interruptEnabler;
        EventManager::run<sampler, handler>();
    }
}

void assertFunction(bool b, const char* function, const char* file, unsigned int line) {
    if (!b) {
        std::cout << "Assertion failed: " << function << "," << file << "," << line << std::endl;
        while(true) {}
    }
}

ISR(TIMER1_COMPA_vect) {
    isrReg::isr<AVR::ISR::Timer<1>::CompareA>();
}
ISR(TIMER1_COMPB_vect) {
    isrReg::isr<AVR::ISR::Timer<1>::CompareB>();
}
ISR(TIMER1_CAPT_vect) {
    isrReg::isr<AVR::ISR::Timer<1>::Capture>();
}
ISR(SPI_STC_vect) {
    isrReg::isr<AVR::ISR::Spi<0>::Stc>();
}
ISR(TIMER0_COMPA_vect) {
    isrReg::isr<AVR::ISR::Timer<0>::CompareA>();
}
ISR(USART_RX_vect) {
//    isrReg::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART_UDRE_vect){
//    isrReg::isr<AVR::ISR::Usart<0>::UDREmpty>();
}
