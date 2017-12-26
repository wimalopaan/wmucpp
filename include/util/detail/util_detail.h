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

#include "util/concepts.h"
#include "container/pgmarray.h"

namespace Util {
    
    template<std::Integral T, uint8_t Base = 10> constexpr uint8_t numberOfDigits();
    
    namespace detail {
        
        template<uint8_t Position, typename T, uint16_t L>
        auto ftoa(T& v, std::array<char, L>& data) -> decltype(data)& {
            typedef typename Util::fragmentType<T>::type FT;
            v *= 10;
            if (v != 0) {
                data[Position] = '0' + (v >> ((sizeof(FT)) * 8));
                v &= FT(-1);
                if constexpr(Position < L - 2) {
                    return ftoa<Position + 1>(v, data);
                }
            }
            return data;    
        }
        
        template<uint8_t Digits = 2, uint8_t Base = 10> 
        struct Convert {
            static_assert(Base <= 16, "wrong Base");
            static_assert(Digits <= 4, "wrong Digits");
            
            typedef uint16_t dimension_type;
            constexpr inline static dimension_type dimension = Base * Base;
            
            static constexpr char toChar(uint8_t d) { // todo: only [0,Base-1]
                static_assert(Base <= 16, "wrong Base");
                if constexpr(Base > 10) {
                    if (d < 10) {
                        return '0' + d;
                    }        
                    else {
                        return 'a' + d - 10;
                    }
                }
                else {
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
            
//            constexpr static inline auto lookupTable = Generator()();
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
        };
        
        template<uint8_t Base, std::Unsigned T>
        void itoa(T value, uint8_t length, char* data) {
            auto next = length - 1;
            constexpr auto modul = detail::Convert<2, Base>::dimension;
            while(value >= modul) {
                auto const d = value % modul;
                data[next--] = detail::Convert<2, Base>::lookupTable[d * 2 + 1];
                data[next--] = detail::Convert<2, Base>::lookupTable[d * 2 + 0];
//                data[next--] = detail::Convert<2, Base>::lookupTable[d][1];
//                data[next--] = detail::Convert<2, Base>::lookupTable[d][0];
                value /= modul;
            }
            if (value < Base) {
                data[next] = detail::Convert<2, Base>::toChar(value);
            }
            else {
                auto const d = (uint8_t)value;
                data[next--] = detail::Convert<2, Base>::lookupTable[d * 2 + 1];
                data[next] = detail::Convert<2, Base>::lookupTable[d * 2 + 0];
//                data[next--] = detail::Convert<2, Base>::lookupTable[d][1];
//                data[next] = detail::Convert<2, Base>::lookupTable[d][0];
            }
        }
        
        
        template<uint8_t Base, std::Integral T, Util::Subscriptable C>
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
        
        template<int Position, uint8_t Base, std::Integral T, uint16_t L>
        uint8_t itoa_single(T& value, std::array<char, L>& data) {
            static_assert((Position < 0) || (Position < L), "wrong length");
            if constexpr(Position >= 0) {
                uint8_t fraction = value % Base;
                data[Position] = Convert<1, Base>::toChar(fraction);
                value /= Base;
                return itoa_single<Position - 1, Base, T>(value, data);
            }
            return 0;
        }

        template<int Position, uint8_t Base, std::Integral T>
        uint8_t itoa_single(T& value, char* data) {
            if constexpr(Position >= 0) {
                uint8_t fraction = value % Base;
                data[Position] = Convert<1, Base>::toChar(fraction);
                value /= Base;
                return itoa_single<Position - 1, Base, T>(value, data);
            }
            return 0;
        }
        
    } // detail
} // Util
