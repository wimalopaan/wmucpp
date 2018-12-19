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

//template<typename T, uint8_t Bits = (sizeof(T) * 8)> struct Fraction;

namespace etl {
    using namespace etl::Concepts;
      
    namespace detail {
        
        template<uint8_t Digits = 2, uint8_t Base = 10> 
        struct Convert {
            static_assert(Base <= 16, "wrong Base");
            static_assert(Digits <= 4, "wrong Digits");
            
            typedef uint16_t dimension_type;
            constexpr inline static dimension_type dimension = Base * Base;
            
            static inline constexpr char toChar(uint8_t d) { // todo: only [0,Base-1]
                static_assert(Base <= 16, "wrong Base");
                if constexpr(Base > 10) {
                    if (d < 10) {
                        return '0' + d;
                    }        
                    else {
                        return 'a' + d - 10;
                    }
                }
                else { // Base <= 10
                    return '0' + d;
                }
            }

            struct Generator {
                constexpr auto operator()() {
                    std::array<char, Digits * dimension> data;
                    for(dimension_type i = 0; i < dimension; ++i) {
                        auto value = i;
                        for(int8_t d = Digits - 1; d >= 0; --d) {
                            auto r = value % Base;
                            data[d + i * Digits] = toChar(r);
                            value /= Base;
                        }
                    }
                    return data;
                }  
            };
            using t = typename ::Util::Pgm::Converter<Generator>::pgm_type;
            
            constexpr static inline auto lookupTable = t{};
            
            template<typename T>
            constexpr static inline uint8_t maxPower = [](){
                uint64_t v = 1;
                for(uint8_t i = 0; i < 64; ++i) {
                    if (v >= std::numeric_limits<T>::max() / Base) {
                        return i;
                    }
                    v *= Base;
                }
            }();
            
            template<typename T>
            constexpr static inline auto powers = [](){
                std::array<uint64_t, maxPower<T> + 1> data;
                uint64_t v = 1;
                for(auto& p : data) {
                    p = v;
                    v *= Base;
                }
                return data;
            }();
            
            template<uint8_t B, uint8_t E, typename T>
            constexpr static uint8_t digits_r(T v) {
                constexpr uint8_t mid = (B + E) / 2;
                if constexpr(mid == B) {
                    return mid + 1;
                }
                
                if (v < powers<T>[mid]) {
                    return digits_r<B, mid>(v);
                }
                else {
                    return digits_r<mid, E>(v);
                }
            }
            template<typename T>
            constexpr static uint8_t digits(T v) {
                return digits_r<0, powers<T>.size - 1>(v);
            }
        }; // class Convert
        
        template<int Position, uint8_t Base, Integral T>
        inline constexpr uint8_t itoa_single(T& value, char* data) {
            if constexpr(Position >= 0) {
                uint8_t fraction = value % Base;
                data[Position] = Convert<1, Base>::toChar(fraction);
                value /= Base;
                if (value == 0) {
                    return Position;
                }
                return itoa_single<Position - 1, Base, T>(value, data);
            }
            return 0;
        }
        
        template<int Position, uint8_t Base, Integral T, uint16_t L>
        inline constexpr uint8_t itoa_single(T& value, std::array<char, L>& data) {
            static_assert((Position < 0) || (Position < L), "wrong length");
            return itoa_single<Position, Base>(value, &data[0]);
        }
        
        template<uint8_t Base, Unsigned T>
        inline constexpr void itoa(T value, uint8_t length, char* data) {
            auto next = length - 1;
            constexpr auto modul = etl::detail::Convert<2, Base>::dimension;
            while(value >= modul) {
                auto const d = value % modul;
                data[next--] = etl::detail::Convert<2, Base>::lookupTable[d * 2 + 1];
                data[next--] = etl::detail::Convert<2, Base>::lookupTable[d * 2 + 0];
                value /= modul;
            }
            if (value < Base) {
                data[next] = etl::detail::Convert<2, Base>::toChar(value);
            }
            else {
                auto const d = (uint8_t)value;
                data[next--] = etl::detail::Convert<2, Base>::lookupTable[d * 2 + 1];
                data[next] = etl::detail::Convert<2, Base>::lookupTable[d * 2 + 0];
            }
        }
        
        template<uint8_t Base, Integral T, typename C>
        constexpr auto itoa(const T& value, C& data) -> decltype(data)& {
            T v = value;
            if constexpr(std::is_signed<T>::value) {
                if (value < 0) {
                    v = -value; 
                }
            }
            uint8_t position = std::numeric_limits<uint8_t>::max();
            do {
                uint8_t fraction = v % Base;
                data[++position] = Convert<1, Base>::toChar(fraction);
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

        template<uint8_t Position, typename T, typename C>
        auto ftoa(T& v, C& data) -> decltype(data)& {
            typedef fragmentType_t<T> FT;
            v *= 10;
            if (v != 0) {
                data[Position] = '0' + (v >> ((sizeof(FT)) * 8));
                v &= FT(-1);
                if constexpr(Position < C::size() - 2) {
                    return ftoa<Position + 1>(v, data);
                }
            }
            return data;    
        }
    } // detail
    
    template<uint8_t Base = 10, Integral T = uint8_t, uint16_t L>
    auto itoa_r(T value, std::array<char, L>& data) -> decltype(data)& {
        static_assert((Base >= 2) && (Base <= 16), "wrong base");
        static_assert(L > numberOfDigits<T, Base>(), "wrong length");
        constexpr uint8_t Position = numberOfDigits<T, Base>() - 1;
        T v = value;
        if constexpr(std::is_signed<T>::value) {
            if (value < 0) {
                v = -value; 
            }
            uint8_t last = etl::detail::itoa_single<Position, Base, T>(v, data);
            if (value < 0) {
                data[last] = '-';
            }
        }   
        else {
            etl::detail::itoa_single<Position, Base, T>(v, data);
        }
        return data;
    }
    
    template<uint8_t Base = 10, Integral T = uint8_t>
    void itoa_r(T value, char* data) {
        static_assert((Base >= 2) && (Base <= 16), "wrong base");
        constexpr uint8_t Position = numberOfDigits<T, Base>() - 1;
        T v = value;
        if constexpr(std::is_signed<T>::value) {
            if (value < 0) {
                v = -value; 
            }
            uint8_t last = etl::detail::itoa_single<Position, Base, T>(v, data);
            if (value < 0) {
                data[last] = '-';
            }
        }   
        else {
            etl::detail::itoa_single<Position, Base, T>(v, data);
        }
    }
    template<uint8_t Base = 10, Integral T = uint8_t, typename C>
    void itoa_r(T value, C& data) {
        static_assert(C::size > numberOfDigits<T, Base>());
        static_assert((Base >= 2) && (Base <= 16), "wrong base");
        constexpr uint8_t Position = numberOfDigits<T, Base>() - 1;
        T v = value;
        if constexpr(std::is_signed<T>::value) {
            if (value < 0) {
                v = -value; 
            }
            uint8_t last = etl::detail::itoa_single<Position, Base, T>(v, &data[0]);
            if (value < 0) {
                data[last] = '-';
            }
        }   
        else {
            etl::detail::itoa_single<Position, Base, T>(v, &data[0]);
        }
    }
    
    template<uint8_t Base = 10, Integral T = uint8_t, uint16_t L = 0>
    constexpr auto itoa(const T& value, std::array<char, L>& data) -> decltype(data)& {
        static_assert((Base >= 2) && (Base <= 16), "wrong base");
        static_assert(L > numberOfDigits<T, Base>(), "wrong char buffer length");
        return etl::detail::itoa<Base>(value, data);
    }
    
    template<uint8_t Base = 10, Integral T = uint8_t, uint8_t L = 0>
    auto itoa(const T& value, char (&data)[L]) -> decltype(data)& {
        static_assert((Base >= 2) && (Base <= 16), "wrong base");
        static_assert(L > numberOfDigits<T, Base>(), "wrong char buffer length");
        return etl::detail::itoa<Base>(value, data);
    }
    
    template<uint8_t Base = 10, Integral T = uint8_t, typename C>
    auto itoa(const T& value, C& data) -> decltype(data)& {
        static_assert((Base >= 2) && (Base <= 16), "wrong base");
        static_assert(C::size > numberOfDigits<T, Base>(), "wrong char buffer length");
        static_assert(std::is_same<typename C::value_type, char>::value, "not a char container");
        return etl::detail::itoa<Base>(value, data);
    }
    
//    template<typename T, uint16_t L>
//    requires (T::valid_bits > 0)
//    auto ftoa(const T& f, std::array<char, L>& data) -> decltype(data)& {
//        static_assert(L >= (numberOfDigits<T, 10>() + 1), "wrong char buffer length");
//        enclosingType_t<typename T::value_type> v = f.value;
//        data[0] = '.';
//        return etl::detail::ftoa<1>(v, data);    
//    }
    
    template<typename T, Container C>
    requires (T::valid_bits > 0)
    auto ftoa(const T& f, C& data) -> decltype(data)& {
        static_assert(data.size() >= numberOfDigits<T, 10>() + 1, "wrong char buffer length");
        enclosingType_t<typename T::value_type> v = f.value;
        data[0] = '.';
        return etl::detail::ftoa<1>(v, data);    
    }
    
} 
