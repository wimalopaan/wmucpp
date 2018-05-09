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

#include <tuple>
#include <memory>
#include <type_traits>

namespace Meta {
    namespace Tuple::detail {
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
        constexpr void all(const T& tuple, const F& f, std::index_sequence<I...>) {
            (f(std::get<I>(tuple)), ...);
        }
        template<typename T, typename F, size_t... I>
        constexpr void all(T& tuple, const F& f, std::index_sequence<I...>) {
            (f(std::get<I>(tuple)), ...);
        }
        
    }
    template<typename... T, typename F>
    constexpr uint8_t visitAt(const std::tuple<T...>& tuple, uint8_t index, const F& f) {
        return Tuple::detail::visit<sizeof...(T)>::at(tuple, index, f);
    }
    template<typename... T, typename F>
    constexpr uint8_t visitAt(std::tuple<T...>& tuple, uint8_t index, const F& f) {
        return Tuple::detail::visit<sizeof...(T)>::at(tuple, index, f);
    }
    template<typename... T, typename F>
    constexpr void visit(const std::tuple<T...>& tuple, const F& f) {
        Tuple::detail::all(tuple, f, std::make_index_sequence<sizeof...(T)>{});
    }
    template<typename... T, typename F>
    constexpr void visit(std::tuple<T...>& tuple, const F& f) {
        Tuple::detail::all(tuple, f, std::make_index_sequence<sizeof...(T)>{});
    }
}

namespace Util {
    
    template<typename T>
    struct is_tuple : std::false_type {};
    template<typename... II>
    struct is_tuple<std::tuple<II...>> : std::true_type {};
    
    template<typename T>
    constexpr bool isTuple(const T&) {
        typedef std::remove_cv_t<typename std::remove_reference<T>::type> tuple_type;
        return is_tuple<tuple_type>::value;
    }
    
    template<typename T>
    concept bool Tuple() {
        return is_tuple<T>::value;
    }
    
    template<Tuple T>
    constexpr auto size(const T& ) {
        typedef std::remove_cv_t<typename std::remove_reference<T>::type> tuple_type;
        return std::tuple_size<tuple_type>::value;        
    }
    
    namespace detail {
        template<auto... N, typename... II>
        constexpr auto tuple_tail(std::index_sequence<N...>, const std::tuple<II...>& tuple) {
            return std::tuple{std::get<N+1>(tuple)...};
        }
    }
    
    template<typename F, typename... II>
    constexpr auto tuple_tail(const std::tuple<F, II...>& tuple) {
        return detail::tuple_tail(std::make_index_sequence<sizeof...(II)>{}, tuple);
    }
    
    
}
