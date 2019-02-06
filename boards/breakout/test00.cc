/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2019 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <mcu/avr.h>
#include <mcu/internals/systemclock.h>

#include <mcu/internals/cppm.h>
#include <mcu/internals/constantrate.h>

#include <etl/output.h>
#include <etl/fixedpoint.h>
#include <etl/scoped.h>
#include <etl/meta.h>

#include <external/hal/alarmtimer.h>
#include <external/hott/hott.h>
#include <external/bluetooth/roboremo.h>
#include <external/hal/devicemapper.h>

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using PortE = AVR::Port<DefaultMcuType::PortRegister, AVR::E>;

namespace  {
    using namespace std::literals::chrono;
//    constexpr auto interval = 10_ms;
    constexpr auto interval = External::Units::duration_cast<std::chrono::milliseconds>(Hott::hottDelayBetweenBytes);
}

using systemClock = AVR::SystemTimer<0, interval>;
using alarmTimer = External::Hal::AlarmTimer<systemClock>;

int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;
    
    systemClock::init();
//    cppm::init();
    
//    rxSelect::dir<Output>();
//    paired::dir<Input>();
    
//    ppmInPin::dir<Output>(); // debug
    
//    crWriterSensorBinary::init();
//    crWriterSensorText::init();

//    rcUsart::init<115200>();
    
    const auto t = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);

    {
        Scoped<EnableInterrupt<>> ei;
        
        while(true) {
//            ppmInPin::toggle();
//            components::periodic();
            systemClock::periodic([&](){
//                crWriterSensorBinary::rateProcess();
//                crWriterSensorText::rateProcess();
                alarmTimer::periodic([&](alarmTimer::index_type timer){
                    if (timer == t) {
//                        outl<terminal>("c1: "_pgm, sumd::value(1).toInt());    
//                        outl<terminal>("p00: "_pgm, roboremoPA::propValues[0]);    
//                        outl<terminal>("p01: "_pgm, roboremoPA::propValues[1]);    
//                        outl<terminal>("p02: "_pgm, roboremoPA::propValues[2]);    
//                        outl<terminal>("p03: "_pgm, roboremoPA::propValues[3]);    
                    }
                });
            });
        }    
    }
    
}
