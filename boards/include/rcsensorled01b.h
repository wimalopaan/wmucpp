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
#include "mcu/avr/swusart2.h"
#include "mcu/avr/mcuppm.h"
#include "mcu/avr/mcupwm.h"
#include "mcu/avr/adc.h"
#include "hal/constantrate.h"
#include "hal/softspimaster.h"
#include "hal/softppm.h"
#include "external/onewire.h"
#include "external/ds18b20.h"
#include "external/ws2812.h"
#include "external/tle5205.h"
#include "external/rpm.h"
#include "external/hott/hott.h"
#include "external/i2cram.h"
#include "external/gps/gps.h"
#include "external/hx711.h"
#include "hal/alarmtimer.h"
#include "hal/adccontroller.h"

#ifdef MEM
# include "util/memory.h"
#endif

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using PortE = AVR::Port<DefaultMcuType::PortRegister, AVR::E>;

using ledPin = AVR::Pin<PortD, 4>;
using led = WS2812<1, ledPin, ColorSequenceGRB>;
typedef led::color_type Color;

using leds1Pin = AVR::Pin<PortE, 0>;
using leds1 = WS2812<8, leds1Pin, ColorSequenceGRB, false>;
typedef leds1::color_type Color1;
using leds = leds1;

using leds2Pin = AVR::Pin<PortB, 0>;
using leds2 = WS2812<8, leds2Pin, ColorSequenceGRB, false>;
typedef leds2::color_type Color2;

using Tle5205Error = AVR::Pin<PortD, 2>;
using hbridge = TLE5205Hard<0, Tle5205Error>; // timer 0
using hbridge1 = hbridge;

using rpmTimer = AVR::Timer16Bit<3>; // timer 3
constexpr std::RPM MaximumRpm{12000};
constexpr std::RPM MinimumRpm{100};
using rpm = RpmWithIcp<rpmTimer, MinimumRpm, MaximumRpm>;
using rpm1 = rpm;

#ifndef USE_I2C
using SoftSPIData = AVR::Pin<PortC, 4>;
using SoftSPIClock = AVR::Pin<PortC, 5>;
using SoftSPISS = AVR::Pin<PortD, 3>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;
#else 
using TwiMaster = TWI::Master<0>;
using TwiMasterAsync = TWI::MasterAsync<TwiMaster>;

struct MCP23008Parameter {
    static constexpr EventType eventValueAvailable = EventType::I2CRamValueAvailable;
    static constexpr EventType eventError = EventType::I2CRamError;
};
constexpr TWI::Address mcp23008Address{std::byte{39}};
using mcp23008 = I2CGeneric<TwiMasterAsync, mcp23008Address, MCP23008Parameter>;

constexpr TWI::Address oledAddress{std::byte{60}};
using oledEndpoint = detail::SSD1306::TwiEndpoint<TwiMasterAsync, oledAddress>;
using oled = SSD1306<oledEndpoint>;
#endif

using oneWirePin = AVR::Pin<PortD, 7>;
using oneWireMaster = OneWire::Master<oneWirePin, OneWire::Normal>;
using oneWireMasterAsync = OneWire::MasterAsync<oneWireMaster, Hott::hottDelayBetweenBytes, 16, UseEvents<false>>;
using ds18b20 = DS18B20<oneWireMasterAsync, true, UseEvents<false>>;
using ds18b20Sync = DS18B20<oneWireMaster>;

using hardPpm = AVR::PPM<1>; // timer1

using systemClock = AVR::Timer8Bit<2>; // timer 2
using alarmTimer = AlarmTimer<systemClock, UseEvents<false>>;

using adc = AVR::Adc<0, AVR::Resolution<8>>;
using adcController = AdcController<adc, 0, 1, 2, 7, 8>; // 0, 1, 2 : Lipo / 7: ACS-Strom / 8: int. Temperatur

using gpsPA = GPS::GpsProtocollAdapter<0, GPS::VTG, GPS::RMC>;
//using gpsUart = AVR::Usart<0, gpsPA, MCU::UseInterrupts<true>, UseEvents<false>, AVR::ReceiveQueueLength<0>>;

using gpsTx = AVR::Pin<PortB, 7>; // NC
using gpsUartTimer = AVR::Timer16Bit<4>;
using gpsUart = SwUsart::UsartInt<1, gpsTx, gpsPA, gpsUartTimer, 9600>; // Int1 -> PD3

#ifdef USE_HX711
# ifdef USE_HX711_ON_PWM2_DIR2
using hx711_clk = AVR::Pin<PortB, 7>; // Dir2
using hx711_data = AVR::Pin<PortD, 6>; // PWM2
# endif
# ifdef USE_HX711_ON_I2C0
using hx711_clk = AVR::Pin<PortC, 5>; // SCL0
using hx711_data = AVR::Pin<PortC, 4>; // SDA0
# endif
using hx711 = HX711::Sensor<hx711_clk, hx711_data, HX711::Mode::A_Gain128, uint16_t, HX711::UseDelay<false>>;
#endif
