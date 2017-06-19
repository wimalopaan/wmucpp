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

#include <type_traits>
#include <cstddef>

namespace Meta {
    namespace detail {
        struct ListBase {
            typedef ListBase base_type;
        };
        template<typename L, typename T> struct push_front_impl; 
        template<template<typename...> typename L, typename... I, typename T> 
        struct push_front_impl<L<I...>, T> {
            typedef L<T, I...> type;
        };

        template<typename L, typename T> struct push_back_impl; 
        template<template<typename...> typename L, typename... I, typename T> 
        struct push_back_impl<L<I...>, T> {
            typedef L<I..., T> type;
        };
        
        template<typename L> struct front_impl;
        template<template<typename, typename...> typename L, typename F, typename... I>
        struct front_impl<L<F, I...>> {
            typedef F type;  
        };
   
        template<typename L> struct back_impl;
        template<template<typename, typename...> typename L, typename F, typename... I>
        struct back_impl<L<F, I...>> {
            typedef typename back_impl<L<I...>>::type type;
        };
        template<template<typename> typename L, typename B>
        struct back_impl<L<B>> {
            typedef B type;
        };
        
        template<template<typename> typename F, typename L> struct apply_impl;
        template<template<typename> typename F, template<typename...> typename L, typename... I>
        struct apply_impl<F, L<I...>> {
            typedef L<F<I>...> type;                
        };

        template<template<typename> typename P, typename L> struct filter_impl;
        template<template<typename> typename P, template<typename...> typename L> 
        struct filter_impl<P, L<>> {
            typedef L<> type;  
        };
        template<template<typename> typename P, template<typename, typename...> typename L, typename F, typename... I> 
        struct filter_impl<P, L<F, I...>> {
            typedef typename std::conditional<P<F>::value, 
                                    typename push_front_impl<typename filter_impl<P, L<I...>>::type, F>::type, 
                                    typename filter_impl<P, L<I...>>::type
            >::type type;
        };
        template<size_t, typename L> struct nth_element_impl;
        template<template<typename F, typename...> typename L, typename F, typename... I>
        struct nth_element_impl<0, L<F, I...>> {
            typedef F type;  
        };
        template<size_t N, template<typename F, typename...> typename L, typename F, typename... I>
        struct nth_element_impl<N, L<F, I...>> {
            typedef typename nth_element_impl<N-1, L<I...>>::type type;  
        };
        template<typename List, typename T> struct count_impl {
            template<typename U>
            using p = std::is_same<U, T>;
            using filtered = typename filter_impl<p, List>::type;
            inline static constexpr size_t value = filtered::size;
        };
        template<typename List, typename T> struct contains_impl {
            inline static constexpr bool value = (count_impl<List, T>::value > 0);
        };
        
    } // !detail
    namespace concepts {
        template<typename T>
        concept bool List() {
            return requires {
                typename T::base_type;  
            } && std::is_same<typename T::base_type, detail::ListBase>::value;
        }
    } // !concepts
    
    template<typename... T>
    struct List : public detail::ListBase {
        inline static constexpr size_t size = sizeof...(T);
    };
    
    template<concepts::List List>
    struct Size : public::std::integral_constant<size_t, List::size> {};
    
    template<concepts::List List, typename T>
    using push_front = typename detail::push_front_impl<List, T>::type;

    template<concepts::List List, typename T>
    using push_back = typename detail::push_back_impl<List, T>::type;
    
    template<concepts::List List>
    using front = typename detail::front_impl<List>::type;

    template<concepts::List List>
    using back = typename detail::back_impl<List>::type;
    
    template<template<typename> typename Func, concepts::List List>
    using apply = typename detail::apply_impl<Func, List>::type;

    template<template<typename> typename Pred, concepts::List List>
    using filter = typename detail::filter_impl<Pred, List>::type;
    
    template<size_t N, concepts::List List>
    using nth_element = typename detail::nth_element_impl<N, List>::type;
    
    template<concepts::List List, typename T>
    struct contains : public std::integral_constant<bool, detail::contains_impl<List, T>::value> {};

    template<concepts::List List, typename T>
    struct count: public std::integral_constant<size_t, detail::count_impl<List, T>::value> {};
    
    namespace tests {
        struct A {};
        struct B {};
        struct C {};
        struct D {};
        
        using l1 = Meta::List<A, B>;
        static_assert(l1::size == 2);
        static_assert(Size<l1>::value == 2);
        
        using l2 = Meta::push_front<l1, C>;
        static_assert(l2::size == 3);
        
        using x1 = Meta::front<l2>;
        static_assert(std::is_same<x1, C>::value);
        
        template<typename T> using add_pointer = T*;
        using l3 = Meta::apply<add_pointer, l2>;
        using h3 = Meta::front<l3>;
        static_assert(std::is_same<h3, C*>::value);     
        
        template<typename T> using p1 = std::is_same<A, T>;
        using l4 = Meta::filter<p1, l1>;
        static_assert(l4::size == 1);     
        using h4 = Meta::front<l4>;
        static_assert(std::is_same<h4, A>::value);
    
        using l5 = Meta::List<A, B, A, C, D>;
        using l6 = Meta::filter<p1, l5>;
        static_assert(l6::size == 2);     
        using h6 = Meta::front<l6>;
        static_assert(std::is_same<h6, A>::value);
        using e0 = Meta::nth_element<0, l6>;
        static_assert(std::is_same<e0, A>::value);
        using e1 = Meta::nth_element<1, l6>;
        static_assert(std::is_same<e1, A>::value);
        
        using b6 = Meta::back<l6>;
        static_assert(std::is_same<b6, A>::value);
        using b1 = Meta::back<l1>;
        static_assert(std::is_same<b1, B>::value);
        
        constexpr bool t1 = contains<l1, A>::value;
        static_assert(t1);
        constexpr bool t2 = contains<l1, D>::value;
        static_assert(!t2);
        
        using l10 = Meta::push_back<l1, D>;
        using b10 = Meta::back<l10>;
        static_assert(std::is_same<b10, D>::value);
        
    } // !tests
} // !Meta