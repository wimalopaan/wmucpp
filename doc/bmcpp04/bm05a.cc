/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <stdint.h>
#include "std/array.h"

typedef uint32_t size_t;

namespace std { 

template<typename T, T... I>
struct integer_sequence{
    using type = T;
    static constexpr T size = sizeof...(I);
    /// Generate an integer_sequence with an additional element.
    template<T N>
    using append = integer_sequence<T, I..., N>;
    
    using next = append<size>;
};
template<typename T, T... I>
constexpr T integer_sequence<T, I...>::size;

template<size_t... I>
using index_sequence = integer_sequence<size_t, I...>;

namespace detail {
template<typename T, T Nt, size_t N>
struct iota {
    static_assert( Nt >= 0, "N cannot be negative" );   
    using type = typename iota<T, Nt-1, N-1>::type::next;
};

template<typename T, T Nt>
struct iota<T, Nt, 0ul> {
    using type = integer_sequence<T>;
};
}

// make_integer_sequence<T, 2> := iota<T, 2, 2>::type
//                             := iota<T, 1, 1>::type::next
//                             := iota<T, 0, 0>::type::next::next
//                             := integer_sequence<T>::next::next
//                             := integer_sequence<T>::append<0>::next
//                             := integer_sequence<T, 0>::next
//                             := integer_sequence<T, 0>::append<1>
//                             := integer_sequence<T, 0, 1>

// make_integer_sequence<T, 1> := iota<T, 1, 1>::type
//                             := iota<T, 0, 0>::type::next
//                             := integer_sequence<T>::next
//                             := integer_sequence<T>::append<0>
//                             := integer_sequence<T, 0>

// make_integer_sequence<T, N> is an alias for integer_sequence<T, 0,...N-1>
template<typename T, T N>
using make_integer_sequence = typename detail::iota<T, N, N>::type;

template<int N>
using make_index_sequence = make_integer_sequence<size_t, N>;

template<typename... Args>
using index_sequence_for = make_index_sequence<sizeof...(Args)>;
}

constexpr uint8_t foo(size_t v) {
    return v + 0x30;
}

template<typename ItemType, typename F, size_t... II>
constexpr auto createArray(F f, std::index_sequence<II...>) {
    return std::array<ItemType, sizeof...(II)> {{f(II)...}};
}

int main()
{
    constexpr int Size = 10;
    std::array<uint8_t, Size> values = {1, 1, 1};
    
    constexpr auto indices = createArray<uint8_t>([](size_t v){return v + 1;}, std::make_index_sequence<3>{});
//    constexpr auto indices = []<typename T, typename F, size_t... II>(T t, F f, std::index_sequence<II...>){std::array<T, sizeof...(II)> {{f(II)...}}}(int(0), [](size_t v){return v + 1;}, std::make_index_sequence<3>{});

//    auto x = []<typename T>(T p){return p;};
    
    uint8_t sum = 0;
    for(const auto& i : indices) {
        sum += values[i];
    }
    return sum;    
}

constexpr void assertFunction(bool b, const char*, const char*, unsigned int) {
        if (!b) {
    //        std::cout << "Assertion failed in: " << function << " file: " << file << " line: " << line << std::endl;
        }
}

