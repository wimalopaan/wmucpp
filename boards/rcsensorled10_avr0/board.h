
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
#include <mcu/internals/cppm.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/constantrate.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/adc.h>

#include <etl/fixedpoint.h>
#include <etl/scoped.h>
#include <etl/meta.h>
#include <etl/converter.h>
#include <etl/output.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/devicemapper.h>
#include <external/hal/adccontroller.h>
//#include <external/hal/pwm.h>

#include <external/hott/hott.h>
#include <external/hott/menu.h>
#include <external/bluetooth/roboremo.h>
#include <external/onewire/onewire.h>

#include <external/solutions/rpm.h>
#include <external/solutions/gps.h>
#include <external/solutions/ws2812.h>
#include <external/solutions/ds18b20.h>
#include <external/solutions/tle5205.h>

#include <external/ui/menu.h>

#ifdef MEM
# include "util/memory.h"
#endif
 
using PortB = AVR::Port<AVR::B>;
using PortC = AVR::Port<AVR::C>;
using PortD = AVR::Port<AVR::D>;
using PortE = AVR::Port<AVR::E>;

#ifndef HOTT_NEW
struct AsciiHandler;
struct BinaryHandler;
struct BCastHandler;
#endif

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

#ifndef HOTT_NEW
using sensorPA = Hott::SensorProtocollAdapter<0, sensorCode, AsciiHandler, BinaryHandler, BCastHandler>;
//using roboremoPA = External::RoboRemo::ProtocollAdapter<0, 16>;
using sensorUsart = AVR::Usart<AVR::Component::Usart<0>, sensorPA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>> ;
using sensorData = Hott::GamProtocollBuffer<0>;
using crWriterSensorBinary = ConstanteRateWriter<sensorData, sensorUsart>;
using menuData = Hott::SensorTextProtocollBuffer<sensorCode, 0>;
using crWriterSensorText = ConstanteRateWriter<menuData, sensorUsart>;
#endif

//using btUsart = AVR::Usart<0, roboremoPA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>> ;

using sumd = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
using rcUsart = AVR::Usart<AVR::Component::Usart<1>, sumd, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

using gpsPA = External::GPS::GpsProtocollAdapter<0, External::GPS::VTG, External::GPS::RMC>;
using gpsUsart = AVR::Usart<AVR::Component::Usart<2>, gpsPA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

using terminalDevice = rcUsart;;
using terminal = etl::basic_ostream<terminalDevice>;

using systemClock = AVR::SystemTimer<AVR::Component::Timer<2>, interval>;
using alarmTimer = External::Hal::AlarmTimer<systemClock>;

using oneWirePin = AVR::Pin<PortE, 0>;
using oneWireMaster = OneWire::Master<oneWirePin, OneWire::Normal>;
using oneWireMasterAsync = OneWire::MasterAsync<oneWireMaster, Hott::hottDelayBetweenBytes, 16>;
using ds18b20 = External::DS18B20<oneWireMasterAsync, true>;
using ds18b20Sync = External::DS18B20<oneWireMaster>;

using ledPin = AVR::Pin<PortC, 0>;
using leds = External::WS2812<9, ledPin, External::ColorSequenceGRB>;

typedef leds::color_type Color;

using ledPin2 = AVR::Pin<PortC, 1>;
using leds2 = External::WS2812<8, ledPin2, External::ColorSequenceGRB>;

using adc = AVR::Adc<AVR::Component::Adc<0>, AVR::Resolution<10>, AVR::AD::VRef<AVR::AD::V2_56>>;
using adcController = External::Hal::AdcController<adc, 0, 1, 2, 3, 4, 5, 6, 7>;
using cellVoltageConverter = Hott::Units::Converter<adc, Hott::Units::cell_voltage_t>;
using battVoltageConverter = Hott::Units::Converter<adc, Hott::Units::battery_voltage_t>;


using hardPwm = AVR::PWM::StaticPwm<AVR::Component::Timer<0>>; // timer 0
using dir1 = AVR::Pin<PortB, 0>;
using dir2 = AVR::Pin<PortB, 1>;

using hbridge1 = External::TLE5205<hardPwm::Channel<AVR::A>, dir1, sumd::value_type>;
using hbridge2 = External::TLE5205<hardPwm::Channel<AVR::B>, dir2, sumd::value_type>;

using hbrigeError = AVR::Pin<PortB, 2>;

using rpm1 = External::RpmWithIcp<AVR::Component::Timer<1>, MinimumRpm, MaximumRpm>;
using rpm2 = External::RpmWithIcp<AVR::Component::Timer<3>, MinimumRpm, MaximumRpm>;
using rpm3 = External::RpmWithIcp<AVR::Component::Timer<4>, MinimumRpm, MaximumRpm>;

using rpmPrescaler = std::integral_constant<uint16_t, rpm1::prescaler>;
//rpmPrescaler::_;

