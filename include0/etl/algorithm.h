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
#include <algorithm>
#include <array>

#include "meta.h"
#include "types.h"

namespace etl {
    template<typename T>
    struct Intervall {
        const T low;
        const T high;
    };
    
    template<typename T>
    T scale(const T& v, const Intervall<T>& irange, const Intervall<T>& orange) {
        if (v < irange.low) {
            return orange.low;
        }
        else if (v > irange.high) {
            return orange.high;
        }
        else {
            return orange.low + (etl::enclosing_t<T>(v - irange.low) * (orange.high - orange.low)) / (irange.high - irange.low);
        }
    }
    
    template <typename T, auto N, typename Comp = std::greater<T>>
    inline constexpr std::array<T, N>& sort(std::array<T, N>& array, Comp compare = std::greater<T>()) {
        for(uint8_t i = 0; i < (N - 1);) {
            if (compare(array[i], array[i + 1])) {
                using std::swap;
                swap(array[i], array[i + 1]);
                i = 0;
            }
            else {
                ++i;
            }
        }
        return array;
    }

    //    template<typename T>
//    class reverse_iterator {
//    public:
//        constexpr reverse_iterator(const T* p) : mActual(p) {}
        
//        constexpr bool operator!=(const reverse_iterator& rhs) {
//            return mActual != rhs.mActual;
//        }
//        constexpr reverse_iterator& operator++() {
//            --mActual;
//            return *this;
//        }
//        constexpr const T& operator*() const {
//            return *mActual;
//        }
        
//    private:
//        const T* mActual = 0;
//    };
    
//    template<typename T, uint32_t N> 
//    constexpr reverse_iterator<T> rbegin( T (&array)[N] ) {
//        return reverse_iterator<T>(&array[N-1]);
//    }
    
//    template<typename T, uint32_t N> 
//    constexpr reverse_iterator<T> rend( T (&array)[N] ) {
//        return reverse_iterator<T>(&array[0] - 1);
//    }
    
//    template<typename C>
//    class reverse_container {
//    public:
//        constexpr reverse_container(const C& c) : mContainer(c) {}
        
//        constexpr auto begin() {
//            return rbegin(mContainer);
//        }
//        constexpr auto end() {
//            return rend(mContainer);
//        }
        
//    private:
//        const C& mContainer;
//    };
    
//    template<typename C>
//    constexpr reverse_container<C> reverse(const C& container) {
//        return reverse_container<C>(container);
//    }
    
    template<etl::Concepts::Container C>
    constexpr bool inline crc8(const C& data) {
        uint8_t crc = 0;
        for(typename C::size_type loop_count = 0; loop_count < data.size(); loop_count++) {
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
    
    constexpr inline void crc16(uint16_t& crc, uint8_t value) {
        constexpr uint16_t crc_polynome = 0x1021;
        crc = crc ^ (((uint16_t)value) << 8);
        for(uint8_t i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ crc_polynome;
            }
            else {
                crc = (crc << 1);
            }
        }
    }

    template<auto... II, typename A, typename B>
    inline static constexpr bool compareElements(const A& a, const B& b) {
        static_assert(((II < std::size(a)) && ...));
        static_assert(((II < std::size(b)) && ...));
        return ((a[II] == b[II]) && ...);
    }
    
    namespace detail {
        template<typename A, typename B, auto... II>
        inline static constexpr bool compareFirstN(const A& a, const B& b, std::index_sequence<II...>) {
            return compareElements<II...>(a, b);
        }
    }
    template<size_t N, typename A, typename B>
    inline static constexpr bool compareFirstN(const A& a, const B& b) {
        return detail::compareFirstN(a, b, std::make_index_sequence<N>{});
    }
    
    template<auto... II, typename A, etl::Concepts::Container B>
    inline static constexpr void copyElements(A& dest, const B& src) {
        static_assert(((II < std::size(dest)) && ...));
        static_assert(((II < std::size(src)) && ...));
        ((dest[II] = typename A::value_type(src[II])),...);
    }
    
    namespace detail {
        template<typename A, etl::Concepts::Container B, auto... II>
        inline static constexpr void copyElements(A& a, const B& b, std::index_sequence<II...>) {
            copyElements<II...>(a, b);
        }
        template<typename A, typename... BB, auto... II>
        inline static constexpr void copyElements(A& a, std::index_sequence<II...>, BB... bb) {
            ((a[II] = bb),...);
        }

        template<typename A, typename B, auto... II>
        inline static constexpr bool contains(const B& c, const A& a, std::index_sequence<II...>) {
            return ((c[II] == a) || ...);
        }
    }

    template<auto... II, typename A>
    constexpr bool isInList(const A& item) {
        return ((A{II} == item) || ...);
    }

    template<typename A, etl::Concepts::Container B>
    constexpr bool contains(const B& c, const A& item) {
         return detail::contains(c, item, std::make_index_sequence<c.size()>{});   
    }
    
    template<typename A, etl::Concepts::Container B>
    constexpr void copy(A& dest, const B& src) {
        static_assert(std::size(dest) >= std::size(src));
        detail::copyElements(dest, src, std::make_index_sequence<std::size(src)>{});
    }

    template<typename A, typename... BB>
    constexpr void copy(A& dest, const BB&... bb) {
        static_assert(std::size(dest) >= sizeof...(bb));
        detail::copyElements(dest, std::make_index_sequence<sizeof...(bb)>{}, bb...);
    }
    
    namespace detail {
        template<auto offset, auto... II, typename A, typename B>
        inline constexpr void fill_impl(A& dest, B src, std::index_sequence<II...>) {
            ((dest[II + offset] = src),...);
        }

        template<auto... II, typename A>
        inline constexpr void reverse_impl(A& dest, std::index_sequence<II...>) {
            ((dest[II] = dest[dest.size() - II - 1]),...);
        }
    }
    
    template<etl::Concepts::Container C>
    constexpr void fill(C& c, typename C::value_type v) {
        detail::fill_impl<0>(c, v, std::make_index_sequence<std::size(c)>{});
    }
    template<etl::Concepts::Container C>
    constexpr void fill(C&& c, typename C::value_type v) {
        detail::fill_impl<0>(c, v, std::make_index_sequence<std::size(c)>{});
    }
    
    template<auto offset, etl::Concepts::Container C>
    constexpr void fillOffset(C& c, typename C::value_type v) {
        if constexpr(offset < std::size(c)) {
            detail::fill_impl<offset>(c, v, std::make_index_sequence<std::size(c) - offset>{});
        }
        else {
            (void)v;
        }
    }

    namespace detail {
        template<etl::Concepts::Container C, typename Callable, auto... II>
        constexpr void apply_impl(C& c, Callable f, std::index_sequence<II...>) {
            ((f(c[II])), ...);
        }
    }
    template<etl::Concepts::Container C, typename Callable>
    constexpr void apply(C&& c, Callable f) {
        detail::apply_impl(c, f, std::make_index_sequence<std::size(c)>{});
    }
    
    
    template<etl::Concepts::Container C>
    constexpr void reverse(C& c) {
        detail::reverse_impl(c, std::make_index_sequence<c.size() / 2>{});
    }
    
    template<typename... T>
    constexpr auto maximum(const T&... vv) {
        typedef Meta::front<Meta::List<T...>> t1;
        static_assert(Meta::all_same<t1, Meta::List<T...>>::value);
        
        t1 m = std::numeric_limits<t1>::min();
        
        ((vv > m ? m = vv : vv), ...);
                
        return m;
    }

    template<typename S, typename T1, typename... TT>
    constexpr const T1& select(const S& s, const T1& v0, const TT&... vv) {
        static_assert(Meta::all_same<T1, Meta::List<TT...>>::value);
        
        assert(s < (sizeof...(TT) + 1));
        
        if (s == 0) {
            return v0;
        }
        else {
            if constexpr(sizeof...(TT) > 0) {
                return select(s - 1, vv...);
            }
            else {
                assert(false);
                return v0;
            }
        }
    }
}
