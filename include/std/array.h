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
#include "util/dassert.h"
#include "std/utility.h"

//#include "std/initializer_list.h"

namespace std {

template<typename T, uint8_t Size>
struct array final
{
    typedef T type;
    typedef uint8_t size_type;
    
//    constexpr array() = default;
    
//    template<typename... TT>
//    constexpr array(TT&&... pp) : data{(T)pp...}{
//    }

    constexpr const T* begin() const {
        return &data[0];
    }
    constexpr const T* end() const {
        return &data[Size];
    }
    constexpr T* begin() {
        return &data[0];
    }
    constexpr T* end() {
        return &data[Size];
    }
    constexpr T& operator[](uint8_t index) {
        assert(index < Size);
        return data[index];
    }
    constexpr volatile T& operator[](uint8_t index) volatile {
        assert(index < Size);
        return data[index];
    }
    constexpr const T& operator[](uint8_t index) const {
        assert(index < Size);
        return data[index];
    }
    static constexpr uint8_t size = Size;
//private:
    T data[Size] = {};
};

namespace detail {
template <class T, uint8_t N, size_t... I>
constexpr std::array<std::remove_cv_t<T>, N>
    to_array_impl(T(&a)[N], std::index_sequence<I...>)
{
    return { {a[I]...} };
}
}
 
template <class T, uint8_t N>
constexpr std::array<std::remove_cv_t<T>, N> to_array(T (&a)[N])
{
    return detail::to_array_impl(a, std::make_index_sequence<N>{});
}

template<typename E, typename... Tail>
constexpr auto make_array(E&& f, Tail&&... tail) {
    return std::array<E, sizeof...(Tail) + 1>{f, tail...};
}


}
