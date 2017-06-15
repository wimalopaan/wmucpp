/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "std/array.h"
#include "std/traits.h"
#include "std/memory.h"

#if __has_include(<avr/pgmspace.h>)
# include <avr/pgmspace.h>
#endif

#ifndef __AVR__
# undef PROGMEM
# define PROGMEM
#endif

template<typename T, T... Ts>
class PgmArray final {
    PgmArray() = delete;
    using U = std::remove_const_t<std::remove_reference_t<T>>;
    
    static U value(uint8_t index) {
        if constexpr(std::is_same<uint8_t, T>::value) {
            return pgm_read_byte(&data[index]);
        }
        else {
            std::array<uint8_t, sizeof(T)> bytes;
            for(uint8_t i = 0; i < sizeof(T); ++i) {
                bytes[i] = pgm_read_byte((uint8_t*)(&data[index]) + i);
            }
            return U::createFrom(bytes);
        }
    }

public:
    static constexpr uint8_t size = sizeof... (Ts);
    typedef U type;
    
    class Iterator {
    public:
        constexpr Iterator(uint8_t index = 0) : mIndex(index) {}
        inline U operator*() {
            return value(mIndex);
        }
        inline void operator++() {
            ++mIndex;
        }
        inline bool operator!=(const Iterator& rhs) {
            return mIndex != rhs.mIndex;
        }
    private:
        uint8_t mIndex = 0;
    };
    constexpr Iterator begin() const {
        return Iterator();
    }
    constexpr Iterator end() const {
        return Iterator(size);
    }
    U operator[](uint8_t index) const {
        return value(index);
    }
private:
    inline static constexpr const U data[] PROGMEM = {Ts...}; 
};

