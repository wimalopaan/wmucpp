/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#if __has_include(<avr/io.h>)
# include <avr/io.h>
#endif

#include "../register.h"

namespace AVR {

struct PrescalerPair {
    typedef uint8_t  bits_type;
    typedef uint16_t scale_type;
    uint8_t  bits;
    uint16_t scale;
};

struct A {
    static constexpr char letter = 'A';
};
struct B {
    static constexpr char letter = 'B';
};
struct C {
    static constexpr char letter = 'C';
};
struct D {
    static constexpr char letter = 'D';
};
struct E {
    static constexpr char letter = 'E';
};

}
