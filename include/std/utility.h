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

#pragma once

#include <stdint.h>
#include <stddef.h>

namespace std { 

 
template< class T > struct remove_const          { typedef T type; };
template< class T > struct remove_const<const T> { typedef T type; };
 
template< class T > struct remove_volatile             { typedef T type; };
template< class T > struct remove_volatile<volatile T> { typedef T type; };

template< class T >
struct remove_cv {
    typedef typename std::remove_volatile<typename std::remove_const<T>::type>::type type;
};

template< class T >
using remove_cv_t       = typename remove_cv<T>::type;
template< class T >
using remove_const_t    = typename remove_const<T>::type;
template< class T >
using remove_volatile_t = typename remove_volatile<T>::type;


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
