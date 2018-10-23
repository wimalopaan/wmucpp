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
#include "mcu/avr/cppm.h"
#include "hal/constantrate.h"
#include "external/ws2812.h"
#include "external/vnh2sp30.h"
#include "external/roboremo/roboremo.h"
#include "external/frsky/frsky.h"
#include "external/sbus/sbus.h"
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

using ledPin = AVR::Pin<PortB, 1>;
using led = WS2812<1, ledPin, ColorSequenceRGB>;
typedef led::color_type Color;

// todo: timer 2 und timer 0 tauschen wegen xck2 problem

using hardPwm = AVR::PWM<2>; // timer 2

using hardPwm1 = PwmAdapter<hardPwm, hardPwm::A>;
using hardPwm2 = PwmAdapter<hardPwm, hardPwm::B>;

using systemClock = AVR::Timer8Bit<0>; // timer 0
using alarmTimer = AlarmTimer<systemClock, UseEvents<false>>;

//using adc = AVR::Adc<0, AVR::Resolution<8>, AVR::AD::VRef<AVR::AD::Vextern<2, 500>, DefaultMcuType>>; // todo: geht nicht ?!?
using adc = AVR::Adc<0, AVR::Resolution<8>, AVR::AD::VRef<AVR::AD::V2_56, DefaultMcuType>>;
using adcController = AdcController<adc, 0, 1, 2, 3, 4, 5, 6>;

using RoboRemoPA = RoboRemo::ProtocollAdapter<0>;
using BTUsart = AVR::Usart<0, RoboRemoPA, MCU::UseInterrupts<true>, UseEvents<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<64>>;

using FrSkyPA = FrSky::ProtocollAdapter<0>;
using TeleUsart = AVR::Usart<1, FrSkyPA, MCU::UseInterrupts<true>, UseEvents<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;

using DebugUsart = AVR::Usart<2, void, MCU::UseInterrupts<true>, UseEvents<false>, AVR::ReceiveQueueLength<64>, AVR::SendQueueLength<64>>;

using debugPin = AVR::Pin<PortB, 2>; // I2C Int
using ppmOutTimer = AVR::Timer16Bit<3>; // timer 3
using cppm = AVR::CPPM<3, AVR::B, 8, debugPin>;
