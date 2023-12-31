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

using PortB = AVR::Port<AVR::B>;
using PortC = AVR::Port<AVR::C>;
using PortD = AVR::Port<AVR::D>;
using PortE = AVR::Port<AVR::E>;

using pc0 = AVR::Pin<PortC, 0>;
using pc1 = AVR::Pin<PortC, 1>;

namespace  {
    using namespace std::literals::chrono;
//    constexpr auto interval = 10_ms;
    constexpr auto interval = External::Units::duration_cast<std::chrono::milliseconds>(Hott::hottDelayBetweenBytes);
}

using systemClock = AVR::SystemTimer<AVR::Component::Timer<0>, interval>;
using alarmTimer = External::Hal::AlarmTimer<systemClock>;

int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;
    
    pc0::dir<Output>();
    pc1::dir<Output>();
    
    systemClock::init();

    const auto t = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);

    {
        Scoped<EnableInterrupt<>> ei;
        
        while(true) {
            pc0::toggle();
            systemClock::periodic([&](){
                alarmTimer::periodic([&](alarmTimer::index_type timer){
                    if (timer == t) {
                        pc1::toggle();
                    }
                });
            });
        }    
    }
    
}
