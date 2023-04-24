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
#include <concepts>

namespace etl {
    namespace Concepts {
        template<typename T, typename... ArgType>
        concept Callable = requires(T t) {
                t(ArgType{}...);
            };
        
        template<typename T>
        concept NamedFlag = requires(T) {
                T::value;
                {T::value} -> std::same_as<bool>;
                typename T::value_type;
            };

        template<typename T>
        concept NamedConstant = requires(T) {
                T::value;
                typename T::value_type;
            };

        template<typename T>
        concept Integral = std::is_integral_v<T>;    
        
        template<typename T>
        concept Unsigned = std::is_unsigned_v<T>;    
        
        template<typename T>
        concept Signed = std::is_signed_v<T>;    

        template<typename T>
        concept Arithmetic = std::is_arithmetic_v<T>;    
        
        template<typename R>
        concept Range = requires (R r) { 
                typename R::value_type;
                r.begin();
                r.end();
            };
        
        template<typename R>
        concept Ranged = requires(R r) {
            typename R::value_type;
            R::Lower;
            R::Upper;
        };

        template<typename C>
        concept Container = requires(C c) {
            typename C::value_type;
            c.size();
            c[0];
        };
        
        template<typename T>
        concept Fundamental = std::is_fundamental<T>::value;
        
        template<typename T>
        concept NonFundamental = !std::is_fundamental<T>::value;
    
        template<typename D>
        concept Device = requires(D) {
                D::put(std::byte{0});
                D::get();
        };

        template<typename S>
        concept Stream = requires(S) {
                typename S::device_type;
        };
    }
}

