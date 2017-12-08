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

#define NDEBUG

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "hal/event.h"
#include "hal/alarmtimer.h"
#include "units/duration.h"
#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

// Timer0
using systemTimer = AVR::Timer8Bit<0>;
using alarmTimer = AlarmTimer<systemTimer>;

using testPin = AVR::Pin<PortD, 7>;

const auto periodicTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);

static constexpr auto systemFrequency = 100_Hz;

using terminalDevice = AVR::Usart<0, void, MCU::UseInterrupts<false>>;
using terminal = std::basic_ostream<terminalDevice>;

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(std::byte b) {
        auto timer = std::to_integer<uint7_t>(b);
        if (timer == *periodicTimer) {
            testPin::toggle();
            std::outl<terminal>("tick"_pgm);
        }
        return true;
    }
};



using allEventHandler = EventHandlerGroup<TimerHandler>;

int main() {
    terminalDevice::init<9600>();
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 
    testPin::dir<AVR::Output>();
    testPin::off();

    std::outl<terminal>("Test04"_pgm);
    
    {
        Scoped<EnableInterrupt<>> ei;
        EventManager::run3<allEventHandler>([](){
            terminalDevice::periodic();
            systemTimer::periodic<systemTimer::flags_type::ocfa>([](){
                alarmTimer::periodic();
            });
        });
    }
}




