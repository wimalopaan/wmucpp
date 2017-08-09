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

#include "detail/util_detail.h"

template<typename T> struct Fraction;

namespace Util {
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
    
    template<Util::Fractional T, uint8_t Base = 10>
    constexpr uint8_t numberOfDigits() {
        if constexpr(std::is_same<Fraction<uint8_t>, T>::value) {
            return 10;
        }
        if constexpr(std::is_same<Fraction<uint16_t>, T>::value) {
            return 18;
        }
    }
      
    template<uint8_t Base = 10, std::Integral T = uint8_t, uint16_t L>
    auto itoa_r(T value, std::array<char, L>& data) -> decltype(data)& {
        static_assert((Base >= 2) && (Base <= 16), "wrong base");
        static_assert(L > Util::numberOfDigits<T, Base>(), "wrong length");
        constexpr uint8_t Position = Util::numberOfDigits<T, Base>() - 1;
        T v = value;
        if constexpr(std::is_signed<T>::value) {
            if (value < 0) {
                v = -value; 
            }
            uint8_t last = detail::itoa_single<Position, Base, T>(v, data);
            if (value < 0) {
                data[last] = '-';
            }
        }   
        else {
            detail::itoa_single<Position, Base, T>(v, data);
        }
        return data;
    }
    
    template<uint8_t Base = 10, std::Integral T = uint8_t>
    void itoa_r(T value, char* data) {
        static_assert((Base >= 2) && (Base <= 16), "wrong base");
        constexpr uint8_t Position = Util::numberOfDigits<T, Base>() - 1;
        T v = value;
        if constexpr(std::is_signed<T>::value) {
            if (value < 0) {
                v = -value; 
            }
            uint8_t last = detail::itoa_single<Position, Base, T>(v, data);
            if (value < 0) {
                data[last] = '-';
            }
        }   
        else {
            detail::itoa_single<Position, Base, T>(v, data);
        }
    }
    
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
    namespace V2 {
        template<uint8_t Base = 10, uint8_t offset = 0, std::Unsigned T = uint8_t, uint16_t L = 0>
        void itoa(const T& value, std::array<char, L>& data) {
            static_assert(L >= Util::numberOfDigits<T, Base>(), "wrong length");
            auto length = detail::Convert<2, Base>::digits(value);
            if constexpr(std::is_same<T, uint64_t>::value) {
                using fragmentType = typename Util::fragmentType<T>::type;
                constexpr auto maximumPower = detail::Convert<2, Base>::template maxPower<fragmentType>;
                if (length > maximumPower) {
                    uint32_t v1 = value / detail::Convert<2, Base>::template powers<fragmentType>[maximumPower];
                    uint32_t v2 = value - v1;
                    detail::itoa<Base>(v1 , length, &data[0] + offset);
                    detail::itoa<Base>(v2 , length - maximumPower, &data[0] + offset);
                }
                else {
                    detail::itoa<Base>(value, length, &data[0] + offset);
                }
            }
            else {
                detail::itoa<Base>(value, length, &data[0] + offset);
            }
        }
        
        template<uint8_t Base = 10, std::Signed T = uint8_t, uint16_t L = 0>
        void itoa(const T& value, std::array<char, L>& data) {
            static_assert(L >= Util::numberOfDigits<T, Base>(), "wrong length");
            typedef typename UnsignedFor<T>::type uType;
            if (value < 0) {
                data[0] = '-';
                itoa<Base, 1>(static_cast<uType>(-value), data);
            }
            else {
                itoa<Base>(static_cast<uType>(value), data);
            }
        }
        
    } //!V2
    
    template<Fractional T, uint16_t L>
    auto ftoa(const T& f, std::array<char, L>& data) -> decltype(data)& {
        static_assert(L >= Util::numberOfDigits<T, 10>(), "wrong char buffer length");
        typename Util::enclosingType<typename T::value_type>::type v = f.value;
        data[0] = '.';
        return detail::ftoa<1>(v, data);    
    }
    
    template<Util::Device Device, bool ensure = false>
    void put(const char* str) {
        while(*str) {
            if constexpr(ensure) {
                while(!Device::put(std::byte{*str})) {
                    Util::delay(1_us);
                }
            }
            else {
                Device::put(std::byte{*str});
            }
            ++str;
        }
    }
    
    template<typename Device, bool ensure = false>
    void put(char c) {
        if constexpr(ensure) {
            while(!Device::put(std::byte{c})) {
                Util::delay(1_us);
            }
        }
        else {
            Device::put(std::byte{c});
        }
    }
    
    template<typename Device, bool ensure = false>
    void put(std::byte b) {
        if constexpr(ensure) {
            while(!Device::put(b)) {
                Util::delay(1_us);
            }
        }
        else {
            Device::put(b);
        }
    }
    
    template<Util::Device Device, Util::Array C, bool ensure = false>
    void put(const C& c) {
        for(uint8_t i = 0; i < c.size; ++i) {
            if constexpr(ensure) {
                while(!Device::put(std::byte{c[i]})) {
                    Util::delay(1_us);
                }
            }
            else {
                Device::put(std::byte{c[i]});
            }
        }
    }
    
    template<Util::Device Device>
    void putl(const char* str) {
        Device::put(str);
        Device::put(std::byte{'\n'});
    }
    
    constexpr bool isPowerof2(int v) {
        return v && ((v & (v - 1)) == 0);
    }
    
} // Util
