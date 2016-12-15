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

#define __STDC_LIMIT_MACROS

#include <stdint.h>
//#include "std/types.h"

namespace std {

template<typename T>
struct numeric_limits;

template<>
struct numeric_limits<uint8_t> {
    static constexpr uint8_t max() {return UINT8_MAX;}
    static constexpr uint8_t min() {return 0;}
};

template<>
struct numeric_limits<uint16_t> {
    static constexpr uint16_t max() {return UINT16_MAX;}
    static constexpr uint16_t min() {return 0;}
};

}
