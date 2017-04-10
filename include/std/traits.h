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

namespace std {

#ifndef __GLIBCXX__

//template<typename T>
//struct underlying_type;
/// The underlying type of an enum.
template<typename _Tp>
  struct underlying_type
  {
    typedef __underlying_type(_Tp) type;
  };


template<bool B, class T = void>
struct enable_if {};

template<class T>
struct enable_if<true, T> {
    typedef T type;
};

template<bool B, class T = void>
struct enable_if_not {};

template<class T>
struct enable_if_not<false, T> {
    typedef T type;
};

template<bool B, class T, class F>
struct conditional { 
    typedef T type; 
};
 
template<class T, class F>
struct conditional<false, T, F> { 
    typedef F type; 
};

template<class T, class U>
struct is_same final {
    static constexpr bool value = false;
};
 
template<class T>
struct is_same<T, T> final {
    static constexpr bool value = true;
};

template<typename T>
struct is_integral final {
    static constexpr bool value = false;
};
template<>
struct is_integral<uint8_t> final {
    static constexpr bool value = true;
};
template<>
struct is_integral<int8_t> final {
    static constexpr bool value = true;
};
template<>
struct is_integral<uint16_t> final {
    static constexpr bool value = true;
};
template<>
struct is_integral<int16_t> final {
    static constexpr bool value = true;
};
template<>
struct is_integral<uint32_t> final {
    static constexpr bool value = true;
};
template<>
struct is_integral<int32_t> final {
    static constexpr bool value = true;
};

template<typename T>
struct is_unsigned final {
    static constexpr bool value = (T(0) < T(-1));
};

template<typename T>
struct is_signed final {
    static constexpr bool value = (T(-1) < T(0));
};


#endif
}
