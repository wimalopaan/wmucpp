/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <cstdint>
#include <cstddef>
#include <array>
#include <limits>
#include <type_traits>

#include "concepts.h"
#include "stringbuffer.h"
#include "type_traits.h"
#include "algorithm.h"

namespace etl {
    using namespace etl::Concepts;
    
    namespace detail {
        
        template<auto Base = 10> 
        static inline constexpr Char toChar(uint8_t d) { // todo: only [0,Base-1]
            static_assert(Base <= 16, "wrong Base");
            if constexpr(Base > 10) {
                if (d < 10) {
                    return Char('0' + d);
                }        
                else {
                    return Char('a' + d - 10);
                }
            }
            else { // Base <= 10
                return Char(('0' + d));
            }
        }
        
        template<int Position, uint8_t Base, Integral T, typename C>
        inline constexpr uint8_t itoa_single_impl(T& value, C& data) {
            static_assert((Position < 0) || (Position < data.size()), "wrong length");
            if constexpr(Position >= 0) {
                uint8_t fraction = value % Base;
                data[Position] = toChar<Base>(fraction);
                value /= Base;
                if (value == 0) {
                    return Position;
                }
                return itoa_single_impl<Position - 1, Base, T>(value, data);
            }
            return 0;
        }
        
        template<uint8_t Base, Integral T, typename C>
        inline constexpr auto& itoa_impl(const T& value, C& data) {
            T v = value;
            if constexpr(std::is_signed<T>::value) {
                if (value < 0) {
                    v = -value; 
                }
            }
            uint8_t position = std::numeric_limits<uint8_t>::max();
            do {
                uint8_t fraction = v % Base;
                data[++position] = toChar<Base>(fraction);
                v /= Base;
            } while(v > 0);
            
            if constexpr(std::is_signed<T>::value) {
                if (value < 0) {
                    data[++position] = Char{'-'};
                }
            }    
            //            data[position + 1] = Char{'\0'};    
            std::reverse(&data[0], &data[position]);
//            etl::reverse(data);
            return data;
        }
        
        template<uint8_t Position, typename T, typename C>
        inline constexpr auto& ftoa_impl(T& v, C& data)  {
            typedef fragmentType_t<T> FT;
            v *= 10;
            if (v != 0) {
                data[Position] = Char('0' + (v >> ((sizeof(FT)) * 8)));
                v &= FT(-1);
                if constexpr(Position < data.size() - 2) {
                    return ftoa_impl<Position + 1>(v, data);
                }
            }
            return data;    
        }
    } // detail
    
    template<uint8_t Base = 10, Integral T = uint8_t, typename C>
    inline constexpr auto& itoa_r(T value, C& data)  {
        static_assert((Base >= 2) && (Base <= 16), "wrong base");
        static_assert(data.size() >= numberOfDigits<T, Base>(), "wrong length");
        constexpr uint8_t Position = numberOfDigits<T, Base>() - 1;
        T v = value;
        if constexpr(std::is_signed<T>::value) {
            if (value < 0) {
                v = -value; 
            }
            uint8_t last = etl::detail::itoa_single_impl<Position, Base, T>(v, data);
            if (value < 0) {
                data[last] = Char{'-'};
            }
        }   
        else {
            etl::detail::itoa_single_impl<Position, Base, T>(v, data);
        }
        return data;
    }
    
    template<uint8_t Base = 10, Integral T = uint8_t, typename C>
    inline constexpr auto& itoa(const T& value, C& data) {
        static_assert((Base >= 2) && (Base <= 16), "wrong base");
        static_assert(data.size() >= numberOfDigits<T, Base>(), "wrong char buffer length");
        static_assert(std::is_same<typename C::value_type, Char>::value, "not a char container");
        return etl::detail::itoa_impl<Base>(value, data);
    }
    
    template<typename T, Container C>
    requires (T::valid_bits > 0)
    inline constexpr auto& ftoa(const T& f, C& data)  {
        static_assert(data.size() >= numberOfDigits<T, 10>() + 1, "wrong char buffer length");
        enclosingType_t<typename T::value_type> v = f.value;
        data[0] = Char{'.'};
        return etl::detail::ftoa_impl<1>(v, data);    
    }
    
} 
