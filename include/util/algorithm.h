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

#include <stdint.h>
#include "std/algorithm"
#include "std/array"
#include "std/concepts.h"

#include "util/meta.h"

namespace Util {
    
    //    template <uint8_t I, typename T, typename ...Ts>
    //    struct nth_element_impl {
    //        using type = typename nth_element_impl<I-1, Ts...>::type;
    //    };
    
    //    template <typename T, typename ...Ts>
    //    struct nth_element_impl<0, T, Ts...> {
    //        using type = T;
    //    };
    
    //    template <uint8_t I, typename ...Ts>
    //    using nth_element = typename nth_element_impl<I, Ts...>::type;
    
    template <typename T, auto N, typename Comp = std::greater<T>>
    constexpr std::array<T, N>& sort(std::array<T, N>& array, Comp compare = std::greater<T>()) {
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
    
    
    template<std::Range C>
    constexpr bool inline crc8(const C& data) {
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
    
    template<auto... II, typename A, typename B>
    inline static constexpr void copyElements(A& dest, const B& src) {
        static_assert(((II < std::size(dest)) && ...));
        static_assert(((II < std::size(src)) && ...));
        ((dest[II] = typename A::type(src[II])),...);
    }
    
    namespace detail {
        template<typename A, typename B, auto... II>
        inline static constexpr void copyElements(A& a, const B& b, std::index_sequence<II...>) {
            ::Util::copyElements<II...>(a, b);
        }
    }
    template<typename A, typename B>
    constexpr void copy(A& dest, const B& src) {
        static_assert(std::size(dest) >= std::size(src));
        detail::copyElements(dest, src, std::make_index_sequence<dest.size>{});
    }
    
    namespace detail {
        
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
