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
#include <stddef.h>

#include "std/memory.h"
#include "std/concepts.h"

namespace std {

#ifndef __GLIBCXX__

template<typename T>
struct less {
    constexpr bool operator()(const T &lhs, const T &rhs) const  {
        return lhs < rhs;
    }
};

template<typename T>
struct greater {
    constexpr bool operator()(const T &lhs, const T &rhs) const  {
        return lhs > rhs;
    }
};

template<typename T>
constexpr void swap(T& a, T& b) {
    T tmp = std::move(a);
    a = std::move(b);
    b = std::move(tmp);
}

template<typename C>
constexpr auto begin(const C& c) -> decltype(c.begin()) {
    return c.begin();
}

template<typename C>
constexpr auto begin(C& c) -> decltype(c.begin()) {
    return c.begin();
}

template< class T, size_t N >
constexpr T* begin( T (&array)[N]) {
    return &array[0];
}

template< class T, size_t N >
constexpr T* end( T (&array)[N]) {
    return &array[N];
}

template<typename C>
constexpr auto end(const C& c) -> decltype(c.end()) {
    return c.end();
}

template<typename C>
constexpr auto end(C& c) -> decltype(c.end()) {
    return c.end();
}

template<typename ForwardIt, typename T>
void fill(ForwardIt first, ForwardIt last, const T& value){
    for (; first != last; ++first) {
        *first = value;
    }
}

template<typename IIt, typename OIt>
constexpr OIt copy(IIt first, IIt last, OIt out) {
    while (first != last) {
            *out++ = *first++;
        }
        return out;
}

template <typename C>
constexpr auto size(const C& c) -> decltype(c.size())
{
    return c.size();
}

template <typename T, size_t N>
constexpr size_t size(const T (&array)[N]) noexcept
{
    return N;
}

template<typename T>
constexpr const T& clamp( const T& v, const T& low, const T& high)
{
    return (v < low) ? low : (v > high) ? high : v;
}

template<typename T>
constexpr const T& max(const T& a, const T& b)
{
    return (a < b) ? b : a;
}

template<typename T>
constexpr const T& min(const T& a, const T& b)
{
    return (a < b) ? a : b;
}

template<class ForwardIterator, class T>
void iota(ForwardIterator first, ForwardIterator last, T value)
{
    while(first != last) {
        *first++ = value;
        ++value;
    }
}

template<class ForwardIterator, class T>
void iota(ForwardIterator first, ForwardIterator last, T value, T increment)
{
    while(first != last) {
        *first++ = value;
        value += increment;
    }
}

template<class InputIt, class T>
InputIt find(InputIt first, InputIt last, const T& value)
{
    for (; first != last; ++first) {
        if (*first == value) {
            return first;
        }
    }
    return last;
}

template<class InputIt, class UnaryPredicate>
InputIt find_if(InputIt first, InputIt last, UnaryPredicate p)
{
    for (; first != last; ++first) {
        if (p(*first)) {
            return first;
        }
    }
    return last;
}

template<class InputIt, class UnaryPredicate>
InputIt find_if_not(InputIt first, InputIt last, UnaryPredicate q)
{
    for (; first != last; ++first) {
        if (!q(*first)) {
            return first;
        }
    }
    return last;
}

template< class InputIt, class UnaryPredicate >
bool all_of(InputIt first, InputIt last, UnaryPredicate p)
{
    return std::find_if_not(first, last, p) == last;
}

template< class InputIt, class UnaryPredicate >
bool any_of(InputIt first, InputIt last, UnaryPredicate p)
{
    return std::find_if(first, last, p) != last;
}

template< class InputIt, class UnaryPredicate >
bool none_of(InputIt first, InputIt last, UnaryPredicate p)
{
    return std::find_if(first, last, p) == last;
}

#endif

template<Range C>
bool crc8(const C& data) {
    uint8_t crc = 0;
    for(typename C::size_type loop_count = 0; loop_count < C::size; loop_count++) {
        uint8_t b = data[loop_count];
        uint8_t bit_counter = 8;
		do {
            uint8_t feedback_bit = (crc ^ b) & 0x01;
			if ( feedback_bit == 0x01 ) {
				crc = crc ^ 0x18; //0X18 = X^8+X^5+X^4+X^0
			}
			crc = (crc >> 1) & 0x7F;
			if ( feedback_bit == 0x01 ) {
				crc = crc | 0x80;
			}
			b = b >> 1;
			bit_counter--;
		
		} while (bit_counter > 0);
	}
	return (crc == 0);
}
    


}
