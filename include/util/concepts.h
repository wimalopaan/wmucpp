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

#include <cstdint>
#include <cstddef>

namespace Util {
    
    template<typename A>
    concept bool Array() { 
        return requires (A a) { 
            typename A::value_type;
            a[0];
            a.size;
        };
    }
    
    template<typename S>
    concept bool Subscriptable() { 
        return requires (S s) { 
            s[0];
        };
    }
    
    template<typename T>
    concept bool Fractional = !std::is_integral<T>::value;
    
    template<typename D>
    concept bool Device() {
        return requires(D d) {
            D::put(std::byte{0});
        };
    }
    
    template<typename T>
    concept bool Callable() {
        return requires(T t) {
            t();
        };
    }
    
    
}
