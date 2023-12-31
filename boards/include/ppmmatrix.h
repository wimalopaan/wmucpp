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

namespace AVR {
    template<typename A0, typename A1, typename A2, typename Signal, typename MCU = DefaultMcuType>
    struct Multiplexer {
        constexpr inline static void init() {
            A0::template dir<AVR::Output>();
            A1::template dir<AVR::Output>();
            A2::template dir<AVR::Output>();
            Signal::template dir<AVR::Output>();
            A0::off();
            A1::off();
            A2::off();
            Signal::off();
        }
        constexpr inline static void select(uintN_t<3> number) {
            if (number & 0x01) {
                A0::on();
            }
            else {
                A0::off();
            }
            if (number & 0x02) {
                A1::on();
            }
            else {
                A1::off();
            }
            if (number & 0x04) {
                A2::on();
            }
            else {
                A2::off();
            }
        }
        constexpr inline static void on() {
            Signal::on();
        }
        constexpr inline static void off() {
            Signal::off();
        }
    private:
    };
}


using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using PortE = AVR::Port<DefaultMcuType::PortRegister, AVR::E>;

using ppmInPin  = AVR::Pin<PortE, 2>;
using ppmOutPin = AVR::Pin<PortB, 1>;

using selectA0 = AVR::Pin<PortB, 0>;
using selectA1 = AVR::Pin<PortD, 7>;
using selectA2 = AVR::Pin<PortD, 6>;

using multiplexer = AVR::Multiplexer<selectA0, selectA1, selectA2, ppmOutPin>;

using paired   = AVR::Pin<PortD, 5>;

using rxSelect = AVR::Pin<PortD, 3>;

using cppm = AVR::CPPM<1, AVR::A, 8, ppmInPin>;

using btUsart = AVR::Usart<0> ;
using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>>;

using systemClock = AVR::Timer8Bit<0>; 
using alarmTimer = AlarmTimer<systemClock, UseEvents<false>>;

