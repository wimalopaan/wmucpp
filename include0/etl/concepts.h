/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2019 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

namespace etl {
//    template<Unsigned T, uint8_t Bits> struct Fraction;
    
    namespace Concepts {
        template<typename T, typename... ArgType>
        concept bool Callable = requires(T t) {
                t(ArgType{}...);
            };
        
        template<typename T>
        concept bool NamedFlag = requires(T) {
                T::value;
                {T::value} -> bool;
            };

        template<typename T>
        concept bool NamedConstant = requires(T) {
                T::value;
                {T::value} -> typename T::value_type;
            };

        template<typename T>
        concept bool Integral = std::is_integral<T>::value;    
        
        template<typename T>
        concept bool Unsigned = std::is_unsigned<T>::value;    
        
        template<typename T>
        concept bool Signed = std::is_signed<T>::value;    

        template<typename R>
        concept bool Range = requires (R r) { 
                typename R::value_type;
                r.begin();
                r.end();
            };

        template<typename C>
        concept bool Container = requires(C c) {
            typename C::value_type;
            c.size();
            c[0];
        };
        
        template<typename T>
        concept bool Fundamental = std::is_fundamental<T>::value;
        
        template<typename T>
        concept bool NonFundamental = !std::is_fundamental<T>::value;
    
        template<typename D>
        concept bool Device = requires(D) {
                D::put(std::byte{0});
        };

        template<typename S>
        concept bool Stream = requires(S) {
                typename S::device_type;
        };
    }
}

