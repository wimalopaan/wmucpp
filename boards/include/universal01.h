/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/adc.h"
#include "hal/adccontroller.h"
#include "external/ws2812.h"
#include "external/dcf77.h"

// Post / Pin definitions for board: universal01 (clock)

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using iRPin    = AVR::Pin<PortB, 0>;
using leds1Pin = AVR::Pin<PortB, 1>;
using leds2Pin = AVR::Pin<PortB, 2>;
using ppm1Pin  = AVR::Pin<PortB, 3>;

using adc = AdcController<AVR::Adc<0>, 0>;

using rxdPin         = AVR::Pin<PortD, 0>;
using txdPin         = AVR::Pin<PortD, 1>;
using dcfPin         = AVR::Pin<PortD, 2>;
using ppm2Pin        = AVR::Pin<PortD, 3>;
using powerSwitchPin = AVR::Pin<PortD, 4>;
using spiClockPin    = AVR::Pin<PortD, 5>;
using spiDataPin     = AVR::Pin<PortD, 6>;
using ledPin         = AVR::Pin<PortD, 7>;

using led = WS2812<1, ledPin, ColorSequenceGRB>;
using Color = led::color_type;
