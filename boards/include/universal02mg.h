/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

// Post / Pin definitions for board: universal02 MG (Masse Gedreht)(clock)

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using iRPin          = AVR::Pin<PortB, 0>;
using d0_ss_Pin      = AVR::Pin<PortD, 6>;
using d1_data_Pin    = AVR::Pin<PortD, 5>;
using displayLedsPin = d1_data_Pin;
using d2_clck_Pin    = AVR::Pin<PortB, 3>;
using d3_Pin         = AVR::Pin<PortD, 3>;

using rxdPin         = AVR::Pin<PortD, 0>;
using txdPin         = AVR::Pin<PortD, 1>;
using dcfPin         = AVR::Pin<PortC, 2>;
using powerSwitchPin = AVR::Pin<PortD, 2>;

using ledPin         = AVR::Pin<PortB, 7>;

using led = WS2812<1, ledPin, ColorSequenceGRB>;
using Color = led::color_type;

