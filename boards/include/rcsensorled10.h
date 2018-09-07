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

#include <stdlib.h>

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/pinchange.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/mcuppm.h"
#include "mcu/avr/mcupwm.h"
#include "mcu/avr/adc.h"
#include "mcu/avr/ppm.h"
#include "mcu/avr/swusart2.h"
#include "hal/constantrate.h"
#include "hal/softspimaster.h"
#include "hal/softppm.h"
#include "hal/sumo.h"
#include "hal/multichannelppm.h"
#include "external/onewire.h"
#include "external/ds18b20.h"
#include "external/ws2812.h"
#include "external/rpm.h"
#include "external/hott/hott.h"
#include "external/i2cram.h"
#include "external/vnh2sp30.h"
#include "external/ssd1306.h"
#include "external/gps/gps.h"
#include "external/hx711.h"
#include "hal/alarmtimer.h"
#include "hal/adccontroller.h"
#include "util/converter.h"
#ifdef MEM
# include "util/memory.h"
#endif

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using PortE = AVR::Port<DefaultMcuType::PortRegister, AVR::E>;

using ledPin = AVR::Pin<PortC, 0>;
using leds = WS2812<9, ledPin, ColorSequenceGRB>;
typedef leds::color_type Color;

using ledPin2 = AVR::Pin<PortC, 1>;
using leds2 = WS2812<8, ledPin2, ColorSequenceGRB>;

namespace {
    constexpr std::RPM MaximumRpm{12000};
    constexpr std::RPM MinimumRpm{100};
}

using rpmTimer1 = AVR::Timer16Bit<1>; // timer 1
using rpm1 = RpmWithIcp<rpmTimer1, MinimumRpm, MaximumRpm>;
using rpmTimer2 = AVR::Timer16Bit<3>; // timer 3
using rpm2 = RpmWithIcp<rpmTimer2, MinimumRpm, MaximumRpm>;
using rpmTimer3 = AVR::Timer16Bit<4>; // timer 4
using rpm3 = RpmWithIcp<rpmTimer3, MinimumRpm, MaximumRpm>;

using rpmPrescaler = std::integral_constant<uint16_t, rpm1::prescaler>;
//rpmPrescaler::_;

using TwiMaster = TWI::Master<1>;
using TwiMasterAsync = TWI::MasterAsync<TwiMaster, 64, false>;

struct MCP23008Parameter {
    static constexpr EventType eventValueAvailable = EventType::I2CRamValueAvailable;
    static constexpr EventType eventError = EventType::I2CRamError;
};
constexpr TWI::Address mcp23008Address{std::byte{39}};
using mcp23008 = I2CGeneric<TwiMasterAsync, mcp23008Address, MCP23008Parameter>;

constexpr TWI::Address blmcAddress{std::byte{50}};
using blmc = I2CGeneric<TwiMasterAsync, blmcAddress, void, UseEvents<false>>;

constexpr TWI::Address oledAddress{std::byte{60}};
using oledEndpoint = detail::SSD1306::TwiEndpoint<TwiMasterAsync, oledAddress>;
using oled = SSD1306<oledEndpoint>;

using oneWirePin = AVR::Pin<PortE, 0>;
using oneWireMaster = OneWire::Master<oneWirePin, OneWire::Normal>;
using oneWireMasterAsync = OneWire::MasterAsync<oneWireMaster, Hott::hottDelayBetweenBytes, 16, UseEvents<false>>;
using ds18b20 = DS18B20<oneWireMasterAsync, true, UseEvents<false>>;
using ds18b20Sync = DS18B20<oneWireMaster>;

//#ifdef USE_RPM2_ON_OPTO2
//using softPpm = SoftPPM<AVR::Timer16Bit<4>, AVR::Pin<PortB, 1>, AVR::Pin<PortB, 2>>; // timer 4
//#endif

#ifdef USE_TC1_AS_HARDPPM
using hardPpm1 = AVR::PPM<1, std::integral_constant<uint32_t, rpm1::prescaler>>; // timer1 <-> exclusiv zu rpm1
#endif
#ifdef USE_TC3_AS_HARDPPM
using hardPpm2 = AVR::PPM<3, std::integral_constant<uint32_t, rpm2::prescaler>>; // timer3 <-> exclusiv zu rpm2
#endif

using hardPwm = AVR::PWM<0>; // timer 0
using dir1 = AVR::Pin<PortB, 0>;
using dir2 = AVR::Pin<PortB, 1>;

using hardPwm1 = PwmAdapter<hardPwm, hardPwm::A>;
using hardPwm2 = PwmAdapter<hardPwm, hardPwm::B>;

using hbridge1 = VNH<hardPwm1, dir1>;
using hbridge2 = VNH<hardPwm2, dir2>;

using gpsPA = GPS::GpsProtocollAdapter<0, GPS::VTG, GPS::RMC>;
using gpsUsart = AVR::Usart<2, gpsPA, MCU::UseInterrupts<true>, UseEvents<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;

using hx711_clk = AVR::Pin<PortD, 7>; 
using hx711_data = AVR::Pin<PortC, 2>; 
using hx711 = HX711::Sensor<hx711_clk, hx711_data, HX711::Mode::A_Gain128, uint16_t, HX711::UseDelay<false>>;

// SPI?

using systemClock = AVR::Timer8Bit<2>; // timer 2
using alarmTimer = AlarmTimer<systemClock, UseEvents<false>>;

using adc = AVR::Adc<0, AVR::Resolution<8>>;
using adcController = AdcController<adc, 0, 1, 2, 3, 4, 5, 6, 7>;

#ifdef USE_PPM_ON_OPTO2
# ifdef USE_ICP1
using ppmDecoder = PpmMultiChannelIcp<AVR::Timer16Bit<1>>; // timer 1
# else
using ppmInputPin = AVR::Pin<PortB, 0>; // note: Modifikation des Boards V.03 -> Opto2
using ppmPinSet = AVR::PinSet<ppmInputPin>;
using pinChangeHandlerPpm = AVR::PinChange<ppmPinSet>;
//using ppmDecoder = SumO<pinChangeHandlerPpm, AVR::Timer16Bit<4>>; // timer 4
using ppmDecoder = PpmMultiChannel<pinChangeHandlerPpm, AVR::Timer16Bit<4>>; // timer 4
# endif
#endif
