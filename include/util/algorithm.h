/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include "std/algorithm.h"
#include "std/array.h"

namespace Util {

template <uint8_t I, typename T, typename ...Ts>
struct nth_element_impl {
    using type = typename nth_element_impl<I-1, Ts...>::type;
};

template <typename T, typename ...Ts>
struct nth_element_impl<0, T, Ts...> {
    using type = T;
};

template <uint8_t I, typename ...Ts>
using nth_element = typename nth_element_impl<I, Ts...>::type;

template <typename T, uint8_t N, typename Comp = std::less<T>>
constexpr std::array<T, N> sort(std::array<T, N> array, Comp compare = std::less<T>()) {
    for(uint8_t i = 0; i < (N - 1); ++i) {
        if (!compare(array[i], array[i + 1])) {
            using std::swap;
            swap(array[i], array[i + 1]);
            i = 0;
        }
    }
    return array;
}


template<typename T>
class reverse_iterator {
public:
    constexpr reverse_iterator(const T* p) : mActual(p) {}
    
    constexpr bool operator!=(const reverse_iterator& rhs) {
        return mActual != rhs.mActual;
    }
    constexpr reverse_iterator& operator++() {
        --mActual;
        return *this;
    }
    constexpr const T& operator*() const {
        return *mActual;
    }

private:
    const T* mActual = 0;
};

template<typename T, uint32_t N> 
constexpr reverse_iterator<T> rbegin( T (&array)[N] ) {
    return reverse_iterator<T>(&array[N-1]);
}

template<typename T, uint32_t N> 
constexpr reverse_iterator<T> rend( T (&array)[N] ) {
    return reverse_iterator<T>(&array[0] - 1);
}

template<typename C>
class reverse_container {
public:
    constexpr reverse_container(const C& c) : mContainer(c) {}
    
    constexpr auto begin() {
        return rbegin(mContainer);
    }
    constexpr auto end() {
        return rend(mContainer);
    }
    
private:
    const C& mContainer;
};

template<typename C>
constexpr reverse_container<C> reverse(const C& container) {
    return reverse_container<C>(container);
}

}
