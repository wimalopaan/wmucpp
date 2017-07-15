/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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
#include "std/utility.h"

namespace std {
    
    template<typename... V> struct tuple {};
    template<typename T, typename... V>
    struct tuple<T, V...> : tuple<V...> { 
        constexpr tuple() : tuple<V...>{}, mData{} {}
        constexpr tuple(T t, V... vs) : tuple<V...>{vs...}, mData{t} {}
        template<uint8_t N>
        const auto& get() const {
            if constexpr(N == 0) {
                return mData;
            }
            else {
                return tuple<V...>::template get<N-1>();
            }
        }
        template<uint8_t N>
        auto& get() {
            if constexpr(N == 0) {
                return mData;
            }
            else {
                return tuple<V...>::template get<N-1>();
            }
        }
        T mData;
    };
    template<uint8_t N, typename T, typename... TT>
    constexpr const auto& get(const std::tuple<T, TT...>& tuple) {
        if constexpr(N == 0) {
            return tuple.mData;
        }    
        else {
            const std::tuple<TT...>& base = tuple;
            return get<N - 1>(base);
        }
    }
    template<uint8_t N, typename T, typename... TT>
    constexpr auto& get(std::tuple<T, TT...>& tuple) {
        if constexpr(N == 0) {
            return tuple.mData;
        }    
        else {
            std::tuple<TT...>& base = tuple;
            return get<N - 1>(base);
        }
    }
    
    template<typename... T>
    tuple(T...) -> tuple<T...>;
    
    template <typename>
    struct tuple_size;     
    template <typename... Types>
    struct tuple_size<tuple<Types...>> : public std::integral_constant<size_t, sizeof...(Types)>{};
    
    template<typename T>
    class tuple_size<const T> : public std::integral_constant<size_t, tuple_size<T>::value> {};
       
    namespace detail {
        template <typename Tuple1, size_t... Indices1, typename Tuple2, size_t... Indices2>
        constexpr auto tuple_cat(const Tuple1& tup1, const Tuple2& tup2,
                                  index_sequence<Indices1...>, index_sequence<Indices2...>) {
            return tuple(get<Indices1>(tup1)..., get<Indices2>(tup2)...);
        }
    }
    
    template <typename Tuple1, typename Tuple2>
    constexpr auto tuple_cat(const Tuple1& tup1, const Tuple2& tup2) {
        return detail::tuple_cat(tup1, tup2, make_index_sequence<tuple_size<Tuple1>::value>{}, make_index_sequence<tuple_size<Tuple2>::value>{});
    }
} // !std

namespace Meta {
    namespace detail {
        template<uint8_t N>
        struct visit {
            template<typename T, typename F>
            constexpr static uint8_t at(T& tuple, uint8_t index, const F& f) {
                if (index == (N - 1)) {
                    return f(std::get<N - 1>(tuple));
                }
                else {
                    return visit<N - 1>::at(tuple, index, f);
                }
            }
            
        };
        template<>
        struct visit<0> {
            template<typename T, typename F>
            constexpr static uint8_t at(T&, uint8_t , const F&) {
                assert(false);
                return 0;
            }
        };
        
        template<typename T, typename F, size_t... I>
        void all(const T& tuple, const F& f, std::index_sequence<I...>) {
            (f(std::get<I>(tuple)), ...);
        }
        
    }
    template<typename... T, typename F>
    constexpr uint8_t visitAt(const std::tuple<T...>& tuple, uint8_t index, const F& f) {
        return detail::visit<sizeof...(T)>::at(tuple, index, f);
    }
    template<typename... T, typename F>
    constexpr uint8_t visitAt(std::tuple<T...>& tuple, uint8_t index, const F& f) {
        return detail::visit<sizeof...(T)>::at(tuple, index, f);
    }
    template<typename... T, typename F>
    void visit(const std::tuple<T...>& tuple, const F& f) {
        detail::all(tuple, f, std::make_index_sequence<sizeof...(T)>{});
    }
}

