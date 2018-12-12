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
#include "hal/constantrate.h"
#include "hal/softspimaster.h"
#include "hal/softppm.h"
#include "external/onewire.h"
#include "external/ds18b20.h"
#include "external/ws2812.h"
#include "external/rpm.h"
#include "external/hott/hott.h"
#include "external/i2cram.h"
#include "external/vnh2sp30.h"
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
using led = WS2812<9, ledPin, ColorSequenceGRB>;
typedef led::color_type Color;

using btUsart = AVR::Usart<0> ;
using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>>;

using systemClock = AVR::Timer8Bit<2>; // timer 2
using alarmTimer = AlarmTimer<systemClock>;

