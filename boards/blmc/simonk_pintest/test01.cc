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

#include <etl/output.h>
#include <etl/fixedpoint.h>
#include <etl/scoped.h>
#include <etl/meta.h>

#include <external/hal/alarmtimer.h>
//#include <external/hott/hott.h>
//#include <external/hott/menu.h>

//#include <external/bluetooth/roboremo.h>
//#include <external/bluetooth/qtrobo.h>
//#include <external/solutions/ifx007.h>
//#include <external/solutions/rpm.h>

//#include <external/hal/adccontroller.h>

namespace  {
    using namespace std::literals::chrono;
    using namespace External::Units;
    using namespace External::Units::literals;
//    constexpr auto interval = 16_ms;
    constexpr auto interval = 1_ms;
    constexpr auto fPWM = 4000_Hz;
}

using PortB = AVR::Port<AVR::B>;
using PortC = AVR::Port<AVR::C>;
using PortD = AVR::Port<AVR::D>;

using pinAp = AVR::Pin<PortC, 3>;
using pinBp = AVR::Pin<PortC, 5>;
using pinCp = AVR::Pin<PortD, 4>;

using pinAn = AVR::Pin<PortB, 0>;
using pinBn = AVR::Pin<PortC, 4>;
using pinCn = AVR::Pin<PortD, 5>;

using led = AVR::Pin<PortB, 3>; // MOSI
using dbg2 = AVR::Pin<PortB, 4>; // MISO
using dbg3 = AVR::Pin<PortB, 5>; // SCK

using rcUsart = AVR::Usart<AVR::Component::Usart<0>, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminalDevice = rcUsart;
using terminal = etl::basic_ostream<terminalDevice>;

//using adc = AVR::Adc<AVR::Component::Adc<0>>;
//using adcController = External::Hal::AdcController<adc, Meta::NList<0, 1, 2, 7, 6>>;

using systemClock = AVR::SystemTimer<AVR::Component::Timer<0>, interval>;
using alarmTimer = External::Hal::AlarmTimer<systemClock>;


template<typename... CC>
struct StaticInitializer {
    StaticInitializer() {
        (CC::init(), ...);
    }
};

//using Initializer = StaticInitializer<systemClock, adcController>;
using Initializer = StaticInitializer<systemClock>;

namespace {
    Initializer initializer;
}

int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;

    led::dir<AVR::Output>();
    dbg2::dir<AVR::Output>();
    dbg3::dir<AVR::Output>();

    pinAp::dir<AVR::Output>();
    pinAn::dir<AVR::Output>();
    pinBp::dir<AVR::Output>();
    
    pinBn::dir<AVR::Output>();
    pinCp::dir<AVR::Output>();
    pinCn::dir<AVR::Output>();
    
    rcUsart::init<BaudRate<9600>>();
    
    const auto t1 = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    
    {
        pinAp::on();
        pinBp::on();
        pinCp::on();

        pinAn::off();
        pinBn::off();
        pinCn::off();
        
        pinAn::on();
        pinBn::on();
//        pinCn::on();

//        pinAp::off();
//        pinBp::off();
//        pinCp::off();
        
        while(true) {
            rcUsart::periodic();
            
            systemClock::periodic([&](){

                pinCp::toggleWithOnOff();                

                alarmTimer::periodic([&](alarmTimer::index_type timer){
                    if (timer == t1) {
                        etl::outl<terminal>("test01"_pgm);
                    }
                });
            });
        }
    }
}

