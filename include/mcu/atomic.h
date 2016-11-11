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
#include "std/types.h"

#if __has_include(<avr/builtins.h>)
# include <avr/builtins.h>
#endif

template<typename T>
class AtomicFlag;

template<>
class AtomicFlag<uint4_t> {
public:
    AtomicFlag(bool v = false) : flag{0, v} {}
    inline bool testAndClear() volatile {
        flag.upper = 0;
        flags = __builtin_avr_swap(flags);
        return flag.upper;
    }
    inline void set() volatile {
        flag.lower = 1;
    }
private:
    union {
        uint4_t flag{0, 0};
        uint8_t flags;
    };
};

typedef AtomicFlag<uint4_t> AtomicFlagU4;

template<>
class AtomicFlag<uint8_t> {
public:
    inline bool testAndClear() volatile {
        flags &= 0x0f;
        flags = __builtin_avr_swap(flags);
        return flags & 0xf0;
    }
    inline void set() volatile {
        flags = 0x01;
    }
private:
    uint8_t flags = 0;
};

typedef AtomicFlag<uint8_t> AtomicFlagU8;
