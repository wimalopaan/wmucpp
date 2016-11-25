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
#include "std/limits.h"
#include "util/bits.h"

namespace Util {

template<typename V> struct BufferSize;

template<>
struct BufferSize<uint8_t>{
    BufferSize() = delete;
    static constexpr uint8_t size = 4;
};
template<>
struct BufferSize<int8_t>{
    BufferSize() = delete;
    static constexpr uint8_t size = 5;
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
        if constexpr((N  - 1) > 0) {
            Utoa<N - 1, T, B>::convert(v, buffer);
        }
    }
};

template<typename T>
inline void utoa(T v, std::array<char, BufferSize<T>::size>& buffer) {
    static_assert(std::is_unsigned<T>::value, "must use unsigned type");
    Utoa<BufferSize<T>::size - 1, T, std::array<char, BufferSize<T>::size> >::convert(v, buffer);
}

template<typename T>
inline void itoa(T v, std::array<char, BufferSize<T>::size>& buffer) {
    static_assert(!std::is_unsigned<T>::value, "must use signed type");
    if (v < 0) {
        buffer[0] = '-';
    }
    else {
        buffer[0] = '+';
    }
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

template<typename T>
struct fragmentType;

template<>
struct fragmentType<uint16_t> {
    typedef uint8_t type;
    static constexpr const uint8_t shift = 8;
};

template<>
struct fragmentType<uint32_t> {
    typedef uint16_t type;
    static constexpr const uint8_t shift = 16;
};

template<typename T>
constexpr auto upperHalf(const T& v) -> typename fragmentType<T>::type {
    return v >> fragmentType<T>::shift;
}

template<typename T>
constexpr auto lowerHalf(const T& v) -> typename fragmentType<T>::type {
    return v;
}

template<typename C, typename F>
auto parity(const C& c, const F& f) -> typename C::item_type {
    typename C::item_type parity = 0;
    auto p = (const typename C::item_type*)&c;
    static_assert(sizeof(C) < std::numeric_limits<uint8_t>::max(),"wrong size");
    for(uint8_t i = 0; i < sizeof(C); ++i) {
        f(*p);
        parity += *p++;
    }
    return parity;
}

constexpr bool isPowerof2(int v) {
    return v && ((v & (v - 1)) == 0);
}

}
