/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <stdint.h>
#include "mcu/avr8.h"

// todo: wozu?

namespace AVR {

template<typename T, typename Component>
struct Register {
    static constexpr uint8_t bval0  = 1U;
    static constexpr uint8_t bval1  = 1U << 1U;
    static constexpr uint8_t bval2  = 1U << 2U;
    static constexpr uint8_t bval3  = 1U << 3U;
    static constexpr uint8_t bval4  = 1U << 4U;
    static constexpr uint8_t bval5  = 1U << 5U;
    static constexpr uint8_t bval6  = 1U << 6U;
    static constexpr uint8_t bval7  = 1U << 7U;

    static constexpr auto cAddress = AVR::getBaseAddr<Component>();

};

}
