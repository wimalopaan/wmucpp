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

#include "std/array.h"
#include "mcu/avr/delay.h"

namespace Util {

template<typename V> struct BufferSize;

template<>
struct BufferSize<uint8_t>{
    BufferSize() = delete;
    static constexpr uint8_t size = 4;
};
template<>
struct BufferSize<uint16_t>{
    BufferSize() = delete;
    static constexpr uint8_t size = 6;
};
template<>
struct BufferSize<uint32_t>{
    BufferSize() = delete;
    static constexpr uint8_t size = 12;
};

template<int N, typename T, typename B>
struct Utoa final {
    static inline void convert(T v, B& buffer) {
        buffer[N - 1] = '0' + v % 10;
        v /= 10;
        Utoa<N - 1, T, B>::convert(v, buffer);
    }
};
template<typename T, typename B>
struct Utoa<0, T, B> {
    static inline void convert(T, B& ) {}
};

template<typename T>
inline void utoa(T v, std::array<char, BufferSize<T>::size>& buffer) {
    Utoa<BufferSize<T>::size - 1, T, std::array<char, BufferSize<T>::size> >::convert(v, buffer);
}

template<typename Device, bool ensure = false>
void put(const char* str) {
    while(*str) {
        if (ensure) {
            while(!Device::put(*str)) {
                Util::delay(1_us);
            }
        }
        else {
            Device::put(*str);
        }
        ++str;
    }
}

template<typename Device, bool ensure = false>
void put(char c) {
    if (ensure) {
        while(!Device::put(c)) {
            Util::delay(1_us);
        }
    }
    else {
        Device::put(c);
    }
}

template<typename Device, typename C, bool ensure = false>
void put(const C& c) {
    for(uint8_t i = 0; i < c.size; ++i) {
        if (ensure) {
            while(!Device::put(c[i])) {
                Util::delay(1_us);
            }
        }
        else {
            Device::put(c[i]);
        }
    }
}

template<typename Device>
void putl(const char* str) {
    Device::put(str);
    Device::put('\n');
}

}
