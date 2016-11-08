/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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
#include "optional.h"

struct uint7_t final {
    uint7_t(uint8_t v) : value(v) {}
    uint8_t pad : 1, value : 7;
    operator uint8_t() const {
        return value;
    }
};

namespace std {

template<>
class optional<uint7_t> {
public:
    optional() = default;
    optional(const uint7_t& value) : data{(uint8_t)((value.value & 0x7f) | 0x80)} {}
    explicit operator bool() const {
        return data & 0x80;
    }
    explicit operator bool() {
        return data & 0x80;
    }
    uint7_t operator*() const {
        return uint7_t{(uint8_t)(data & 0x7f)};
    }
private:
    uint8_t data{0};
};

}
