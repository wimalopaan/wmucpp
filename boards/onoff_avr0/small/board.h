
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

#pragma once

#include <mcu/avr.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
//#include <mcu/internals/cppm.h>
//#include <mcu/internals/pwm.h>
//#include <mcu/internals/constantrate.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/adc.h>

//#include <etl/fixedpoint.h>
//#include <etl/scoped.h>
//#include <etl/meta.h>
//#include <etl/converter.h>
//#include <etl/output.h>

#include <external/hal/alarmtimer.h>
//#include <external/hal/devicemapper.h>
#include <external/hal/adccontroller.h>
////#include <external/hal/pwm.h>

#include <external/hott/hott.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>

#include <external/hott/menu.h>
#include <external/bluetooth/roboremo.h>
//#include <external/onewire/onewire.h>

//#include <external/solutions/rpm.h>
//#include <external/solutions/gps.h>
//#include <external/solutions/ws2812.h>
//#include <external/solutions/ds18b20.h>
//#include <external/solutions/tle5205.h>

//#include <external/ui/menu.h>

#ifdef MEM
# include "util/memory.h"
#endif
 
using PortA = AVR::Port<AVR::A>;
//using PortB = AVR::Port<AVR::B>;
//using PortC = AVR::Port<AVR::C>;
//using PortD = AVR::Port<AVR::D>;
//using PortE = AVR::Port<AVR::E>;

namespace  {
    using namespace std::literals::chrono;
//    constexpr auto interval = 10_ms;
//    constexpr auto interval = External::Units::duration_cast<std::chrono::milliseconds>(Hott::hottDelayBetweenBytes);
    constexpr auto interval = Hott::hottDelayBetweenBytes;
    
//    std::integral_constant<uint16_t, interval.value>::_;
    
    constexpr External::Units::RPM MaximumRpm{12000};
    constexpr External::Units::RPM MinimumRpm{100};
    
    constexpr auto sensorCode = Hott::gam_code;
}

//using terminalDevice = sensorUsart;;
//using terminal = etl::basic_ostream<terminalDevice>;

using systemClock = AVR::SystemTimer<AVR::Component::Timer<2>, interval>;
using alarmTimer = External::Hal::AlarmTimer<systemClock>;

using adc = AVR::Adc<AVR::Component::Adc<0>, AVR::Resolution<10>, AVR::AD::VRef<AVR::AD::V2_56>>;
using adcController = External::Hal::AdcController<adc, 0, 1, 2, 3, 4, 5, 6, 7>;
using cellVoltageConverter = Hott::Units::Converter<adc, Hott::Units::cell_voltage_t>;
using battVoltageConverter = Hott::Units::Converter<adc, Hott::Units::battery_voltage_t>;

using sensor = Hott::Experimental::Sensor<AVR::Component::Usart<0>, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, systemClock>;
