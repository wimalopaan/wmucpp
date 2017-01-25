/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

namespace std {

template<typename T>
struct numeric_limits;

template<>
struct numeric_limits<uint8_t> {
    typedef uint8_t type;
    static constexpr uint8_t max() {return 255;}
    static constexpr uint8_t min() {return 0;}
    static constexpr uint16_t module() {return 256;}
};

template<>
struct numeric_limits<uint16_t> {
    typedef uint16_t type;
    static constexpr uint16_t max() {return 65535;}
    static constexpr uint16_t min() {return 0;}
    static constexpr uint32_t module() {return 65536;}
};

}
