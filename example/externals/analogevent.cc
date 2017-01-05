/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "mcu/ports.h"
#include "mcu/avr/adc.h"
#include "hal/softspimaster.h"
#include "hal/adccontroller.h"
#include "hal/alarmtimer.h"
#include "hal/event.h"
#include "external/lm35.h"
#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using systemClock = AVR::Timer8Bit<0>;
using systemTimer = AlarmTimer<systemClock>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

using terminal = SSpi0;

using adc = AVR::Adc<0>;
using adcController = AdcController<adc, 6, 7>;
using lm35 = LM35<adcController, 0>;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

struct TimerHandler : public EventHandler<EventType::Timer> {
    static void process(uint8_t) {
        std::cout << "Value: "_pgm << adcController::value(0) << std::endl;
        std::cout << "Voltage: "_pgm << adcController::voltage(0) << std::endl;
        std::cout << "Temp: "_pgm << lm35::temperature() << std::endl;
    }
};

using pGroup = PeriodicGroup<AVR::ISR::Timer<0>::CompareA, systemTimer, adcController>;
using eGroup = EventHandlerGroup<TimerHandler>;

using isrReg = IsrRegistrar<pGroup>;

int main() {
    terminal::init();
    systemTimer::init();
    
    adcController::init();
    
    std::cout << "Analog Test with Events"_pgm << std::endl;
    
    std::cout << "Channels: "_pgm << adcController::channels[0] << std::endl;
    std::cout << "Channels: "_pgm << adcController::channels[1] << std::endl;
    
    systemTimer::create(1000_ms, AlarmFlags::Periodic);
    
    {
        Scoped<EnableInterrupt> ie;
        EventManager::run<pGroup, eGroup>();
    }

}
ISR(TIMER0_COMPA_vect) {
    isrReg::isr<AVR::ISR::Timer<0>::CompareA>();
//    pGroup::tick();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) {
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif
