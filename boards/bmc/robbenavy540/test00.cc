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
#include <mcu/internals/pwm.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/timer.h>
#include <mcu/internals/softuart.h>

#include <etl/output.h>
#include <etl/fixedpoint.h>
#include <etl/scoped.h>
#include <etl/meta.h>

#include <external/hal/alarmtimer.h>
#include <external/hott/hott.h>
#include <external/hott/menu.h>

#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/solutions/ifx007.h>
#include <external/solutions/rpm.h>

#include <external/hal/adccontroller.h>

namespace  {
    using namespace std::literals::chrono;
    using namespace External::Units;
    using namespace External::Units::literals;
//    constexpr auto interval = 16_ms;
    constexpr auto interval = 16000_us;
    constexpr auto fPWM = 4000_Hz;
}


using PortB = AVR::Port<AVR::B>;
using PortC = AVR::Port<AVR::C>;
using PortD = AVR::Port<AVR::D>;

using pinAn = AVR::ActiveHigh<AVR::Pin<PortD, 2>, AVR::Output>;
using pinBn = AVR::ActiveHigh<AVR::Pin<PortD, 0>, AVR::Output>;
using pinAp = AVR::ActiveLow<AVR::Pin<PortD, 1>, AVR::Output>;
using pinBp = AVR::ActiveLow<AVR::Pin<PortC, 5>, AVR::Output>;

using led = AVR::Pin<PortD, 7>;


int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;

    using pin = pinAn;
    using pip = pinBp;
    
    led::dir<Output>();
    led::on();

    pinAn::init();
    pinBn::init();
    pinAp::init();
    pinBp::init();

    
    {
        while(true) {
            
            pip::activate();
            
            pin::activate();            
            AVR::Util::delay(100_us);
            pin::inactivate();            
            AVR::Util::delay(1_ms);
            
//            systemClock::periodic([&](){
//                alarmTimer::periodic([&](alarmTimer::index_type timer){
//                    if (timer == t) {
//                        led::toggle();
//                    }
//                });
//            });
        }
    }
}

