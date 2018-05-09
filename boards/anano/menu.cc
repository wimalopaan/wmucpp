/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#define NDEBUG

#include "../include/anano.h"

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "hal/event.h"
#include "hal/alarmtimer.h"
#include "units/duration.h"
#include "console.h"

using namespace AVR;

using systemTimer = AVR::Timer8Bit<0>;
using alarmTimer  = AlarmTimer<systemTimer>;

const auto secondsTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);

using terminalDevice = Usart<0, void, MCU::UseInterrupts<false>>;
using terminal = std::basic_ostream<terminalDevice>;

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
    terminalDevice::init<9600>();
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 

    led::dir<AVR::Output>();
    led::on();    

    std::outl<terminal>("Menu"_pgm);
    
    {
        Scoped<EnableInterrupt<>> ei;
        EventManager::run3<allEventHandler>([](){
            terminalDevice::periodic();
            systemTimer::periodic<systemTimer::flags_type::ocfa>([](){
                alarmTimer::rateProcess();
            });
        });
    }
}




