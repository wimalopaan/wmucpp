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

namespace AVR {
    namespace AD {
        struct V1_1;
        struct V2_56;
        
        template<uint16_t Volts, uint16_t MilliVolts>
        struct Vextern {
            static constexpr float value = Volts + (0.001f * MilliVolts);
        };
        template<typename Voltage, typename MCU> struct VRef {
            static constexpr auto refs = typename MCU::Adc::MUX{0};
            static constexpr float value = Voltage::value;
        };
    }
    template<uint8_t TimerN, typename MCU> struct TimerParameter;
    template<uint8_t TimerN, typename MCU> struct Timer8Bit;
    template<uint8_t TimerN, typename MCU> struct Timer16Bit;
}

#include "grmega_x4.h"
#include "grmega_x8.h"
#include "grtiny_x4.h"
#include "grtiny_x5.h"
