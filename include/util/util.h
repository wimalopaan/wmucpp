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

#include <stdint.h>

#include "std/array.h"
#include "mcu/avr/delay.h"
#include "std/limits.h"
#include "std/traits.h"
#include "std/concepts.h"
#include "util/bits.h"
#include "util/concepts.h"
#include "container/stringbuffer.h"

template<typename T> struct Fraction;

namespace Util {

//namespace detail {
//    template<uint8_t N> struct DType;
//    template<> 
//    struct DType<2> {
//        typedef uint16_t dtype;
//        constexpr inline static uint8_t dimension = 100;
//    };
//    template<> 
//    struct DType<4> {
//        typedef uint32_t dtype;
//        constexpr inline static uint16_t dimension = 10000;
//    };
//    template<uint8_t N = 2> 
//    struct DLookupTable {
//        constexpr static inline auto data = [](){
//            std::array<typename DType<N>::dtype ,DType<N>::dimension> data;
//            for(typename DType<N>::dtype i = 0; i < DType<N>::dimension; ++i) {
//                data[i] = ('0' + i % 10) + (('0' + (i / 10)) << 8);
//            }
//            return data;
//        }();
//    };
//}

template<std::Integral T, uint8_t Base = 10>
constexpr uint8_t numberOfDigits() {
    T v = std::numeric_limits<T>::max();
    uint8_t number = 0;
    while(v > 0) {
        v /= Base;
        ++number;
    }
    if (number == 0) {
        number = 1;
    }
    if constexpr(std::is_signed<T>::value) {
        number += 1;
    }
    return number;
}

template<typename T>
concept bool Fractional = !std::is_integral<T>::value;

template<Fractional T, uint8_t Base = 10>
constexpr uint8_t numberOfDigits() {
    if constexpr(std::is_same<Fraction<uint8_t>, T>::value) {
        return 10;
    }
    if constexpr(std::is_same<Fraction<uint16_t>, T>::value) {
        return 18;
    }
}

namespace detail {
template<int Position, uint8_t Base = 10, std::Integral T = uint8_t>
uint8_t itoa_single(T& value, std::array<char, Util::numberOfDigits<T, Base>()>& data) {
    if constexpr(Position >= 0) {
        uint8_t fraction = value % Base;
        if (fraction < 10) {
            data[Position] = '0' + fraction;
        }
        else {
            data[Position] = 'a' - 10 + fraction;
        }
        value /= Base;
        return itoa_single<Position - 1, Base, T>(value, data);
    }
    return 0;
}

} // detail

template<uint8_t Base = 10, std::Integral T = uint8_t>
auto itoa_r(T value, std::array<char, Util::numberOfDigits<T, Base>()>& data) -> decltype(data)& {
    static_assert((Base >= 2) && (Base <= 16), "wrong base");
    T v = value;
    if constexpr(std::is_signed<T>::value) {
        if (value < 0) {
            v = -value; 
        }
    }
    if constexpr(std::is_signed<T>::value) {
        uint8_t last = detail::itoa_single<data.size - 1, Base, T>(v, data);
        if (value < 0) {
            data[last] = '-';
        }
    }   
    else {
        detail::itoa_single<data.size - 1, Base, T>(v, data);
    }
    return data;
}

namespace detail {
template<uint8_t Base = 10, std::Integral T = uint8_t, uint16_t L = 0, typename C = void>
auto itoa(const T& value, C& data) -> decltype(data)& {
    T v = value;
    if constexpr(std::is_signed<T>::value) {
        if (value < 0) {
            v = -value; 
        }
    }
    uint8_t position = std::numeric_limits<uint8_t>::max();
    do {
        uint8_t fraction = v % Base;
        if (fraction < 10) {
            data[++position] = '0' + fraction;
        }
        else {
            data[++position] = 'a' - 10 + fraction;
        }
        v /= Base;
    } while(v > 0);
    
    if constexpr(std::is_signed<T>::value) {
        if (value < 0) {
            data[++position] = '-';
        }
    }    
    data[position + 1] = '\0';    
    std::reverse(&data[0], &data[position]);
    return data;
}
} // detail

template<uint8_t Base = 10, std::Integral T = uint8_t, uint16_t L = 0>
auto itoa(const T& value, std::array<char, L>& data) -> decltype(data)& {
    static_assert((Base >= 2) && (Base <= 16), "wrong base");
    static_assert(L > Util::numberOfDigits<T, Base>(), "wrong char buffer length");
    
    return detail::itoa<Base>(value, data);
}

template<uint8_t Base = 10, std::Integral T = uint8_t, uint8_t L = 0>
auto itoa(const T& value, char (&data)[L]) -> decltype(data)& {
    static_assert((Base >= 2) && (Base <= 16), "wrong base");
    static_assert(L > Util::numberOfDigits<T, Base>(), "wrong char buffer length");
    return detail::itoa<Base>(value, data);
}

template<uint8_t Base = 10, std::Integral T = uint8_t, typename C>
auto itoa(const T& value, C& data) -> decltype(data)& {
    static_assert((Base >= 2) && (Base <= 16), "wrong base");
    static_assert(C::size > Util::numberOfDigits<T, Base>(), "wrong char buffer length");
    static_assert(std::is_same<typename C::value_type, char>::value, "not an char container");
    return detail::itoa<Base>(value, data);
}

namespace detail {

template<uint8_t Position, typename T, uint16_t L>
auto ftoa(T& v, std::array<char, L>& data) -> decltype(data)& {
    typedef typename Util::fragmentType<T>::type FT;
    v *= 10;
    if (v != 0) {
        data[Position] = '0' + (v >> ((sizeof(FT)) * 8));
        v &= FT(-1);
        if constexpr(Position < data.size - 2) {
            return ftoa<Position + 1>(v, data);
        }
    }
    return data;    
}

} // detail

template<Fractional T, uint16_t L>
auto ftoa(const T& f, std::array<char, L>& data) -> decltype(data)& {
    static_assert(L >= Util::numberOfDigits<T, 10>(), "wrong char buffer length");
    typename Util::enclosingType<typename T::value_type>::type v = f.value;
    data[0] = '.';
    return detail::ftoa<1>(v, data);    
}



template<typename Device, bool ensure = false>
void put(const char* str) {
    while(*str) {
        if constexpr(ensure) {
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
    if constexpr(ensure) {
        while(!Device::put(c)) {
            Util::delay(1_us);
        }
    }
    else {
        Device::put(c);
    }
}

template<typename Device, Util::Array C, bool ensure = false>
void put(const C& c) {
    for(uint8_t i = 0; i < c.size; ++i) {
        if constexpr(ensure) {
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

constexpr bool isPowerof2(int v) {
    return v && ((v & (v - 1)) == 0);
}

} // Util
