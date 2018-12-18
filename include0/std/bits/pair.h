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

#include "../cstdint"

namespace std {
    template<typename T, typename U>
    struct pair {
        typedef T first_type;
        typedef U second_type;
        T first;
        U second;
    };
    
//    template<typename T>
//    struct combinedType;
    
//    template<>
//    struct combinedType<uint8_t> {
//        typedef uint16_t type;
//        static constexpr const uint8_t shift = 8;
//    };
    
//    template<typename T>
//    typename combinedType<T>::type combinedValue(volatile const pair<T, T>& p) {
//        return (p.first << combinedType<T>::shift) + p.second;
//    }
    
//    template<typename T>
//    typename combinedType<T>::type combinedValue(const pair<T, T>& p) {
//        return (p.first << combinedType<T>::shift) + p.second;
//    }
    
}
