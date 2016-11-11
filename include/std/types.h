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
#include "std/optional.h"

struct uint4_t {
    uint8_t upper : 4, lower : 4;
};

struct uint7_t final {
    constexpr explicit uint7_t(const uint8_t& v) : pad(0), value(v) {}

    constexpr explicit uint7_t(const uint7_t& v) : pad(0), value(v) {}
    explicit uint7_t(volatile uint7_t& v) : pad(0), value(v) {}

    constexpr uint7_t(uint7_t&&) = default;
    constexpr uint7_t& operator=(uint7_t&&) = default;
    void operator=(uint7_t&& rhs) volatile {
        value = rhs.value;
    }

    uint7_t& operator=(const uint7_t&) = default;

    void operator=(volatile uint7_t& rhs) volatile {
        value = rhs.value;
    }

    constexpr operator uint8_t() const {
        return value;
    }
    operator uint8_t() const volatile {
        return value;
    }
    uint8_t pad : 1, value : 7;
};

namespace std {

template<>
class optional<uint7_t> {
public:
    constexpr optional() = default;
    constexpr optional(uint7_t value) : data{(uint8_t)(value.value | 0x80)} {}
    constexpr explicit operator bool() const {
        return data & 0x80;
    }
    constexpr explicit operator bool() {
        return data & 0x80;
    }
    constexpr uint7_t operator*() const {
        return uint7_t{data};
    }
private:
    uint8_t data{0};
};


}
