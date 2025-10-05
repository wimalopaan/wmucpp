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

#include <cstddef>
#include <utility>
#include <type_traits>
#include <limits>
#include <cassert>
#include <optional>

//#include <etl/ranged.h>

namespace Meta {
    namespace detail {
        struct ListBase {
            typedef ListBase base_type;
        };
    }
    namespace concepts {
        template<typename T>
        concept /*bool */ List = requires(T) {
                typename T::base_type;  
            } && std::is_same<typename T::base_type, detail::ListBase>::value;
    } // !concepts
    
    template<typename... T>
    struct List : public detail::ListBase {
        inline static constexpr size_t size = sizeof...(T);
    };
    
    template<template<typename> typename... TT>
    struct TList {
        inline static constexpr size_t size = sizeof...(TT);
    };

    template<auto... NN>
    struct NList : public detail::ListBase {
        inline static constexpr size_t size = sizeof...(NN);
        typedef List<std::integral_constant<decltype(NN), NN> ...> list_type;
    };
    
    template<typename T> struct Wrapper{
        typedef T type;
    };
    
    namespace detail {
        template<typename> struct front_impl;
        template<typename> struct size_impl;
        template<typename> struct rest_impl;
        template<typename> struct middle_impl;
        template<typename, typename> struct contains_impl;
        template<typename, typename> struct concat_impl;
    }
    
    template<concepts::List List>
    using front = typename detail::front_impl<List>::type;

    template<concepts::List List>
    using size = typename detail::size_impl<List>::type;

    template<concepts::List List>
    inline static constexpr auto size_v = Meta::size<List>::value;
    
    template<typename... T>
    using length = std::integral_constant<size_t, sizeof...(T)>;

    template<concepts::List List>
    using size_type = std::conditional_t<size<List>::value < 256, uint8_t, uint16_t>;    
    
    template<concepts::List List>
    using rest = typename detail::rest_impl<List>::type;

    template<concepts::List List>
    using middle_element = typename detail::middle_impl<List>::type;
    
    template<concepts::List List, typename T>
    struct contains : public std::integral_constant<bool, detail::contains_impl<List, T>::value> {};

    template<concepts::List List, typename T>
    static inline constexpr bool contains_v = contains<List, T>::value;
    
    template<concepts::List L1, concepts::List L2>
    using concat = typename detail::concat_impl<L1, L2>::type;

    template<template<typename...> typename A, template<typename...> typename B>
    struct is_same_template : std::false_type {};
    template<template<typename...> typename T>
    struct is_same_template<T, T> : std::true_type {};

    template<template<typename...> typename A, template<typename...> typename B>
    static inline constexpr bool is_same_template_v = is_same_template<A, B>::value;
    
    namespace detail {
        template<typename L>
        struct size_impl;
        template<template<typename...> typename L, typename... I>
        struct size_impl<L<I...>> {
            using value_type = std::conditional_t<sizeof...(I) < 256, uint8_t, uint16_t>;
            using type = std::integral_constant<value_type, sizeof...(I)>;
        };
        template<auto... I>
        struct size_impl<NList<I...>> {
            using value_type = std::conditional_t<sizeof...(I) < 256, uint8_t, uint16_t>;
            using type = std::integral_constant<value_type, sizeof...(I)>;
        };
        
        template<template<typename...> typename, typename> struct apply_impl;
        template<template<typename...> typename F, template<typename...> typename L, typename... I>
        struct apply_impl<F, L<I...>> {
            typedef typename F<I...>::type type;
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

        template<typename> struct tfront_impl;
        template<template<typename> typename F,
                 template<template<typename> typename...> typename TL,
                 template<typename> typename... R>
        struct tfront_impl<TL<F, R...>> {
            template<typename T> using type = F<T>;
        };
        
        template<typename L> struct rest_impl;
        template<template<typename, typename...> typename L, typename F, typename... I>
        struct rest_impl<L<F, I...>> {
            typedef L<I...> type;  
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
        
        template<template<typename> typename F, typename L> struct transform_impl;
        template<template<typename> typename F, template<typename...> typename L, typename... I>
        struct transform_impl<F, L<I...>> {
            typedef L<F<I>...> type;                
        };

        template<template<typename> typename F, typename L> struct transform_type_impl;
        template<template<typename> typename F, template<typename...> typename L, typename... I>
        struct transform_type_impl<F, L<I...>> {
            typedef Meta::List<typename F<I>::type...> type;                
        };
        
        template<template<typename, size_t> typename F, typename L, typename Seq> struct transformN_impl;
        template<template<typename, size_t> typename F, template<typename...> typename L, typename... I, size_t... II>
        struct transformN_impl<F, L<I...>, std::index_sequence<II...>> {
            static_assert(sizeof...(I) == sizeof...(II));
            typedef L<F<I, II>...> type;                
        };

        template<template<typename, typename> typename F, typename L1, typename L2> struct transform2_impl;
        template<template<typename, typename> typename F, template<typename...> typename L1, typename... I1, template<typename...> typename L2, typename... I2>
        struct transform2_impl<F, L1<I1...>, L2<I2...>> {
            static_assert(sizeof...(I1) == sizeof...(I2));
            typedef L1<F<I1, I2>...> type;
        };
        
        template<template<template<typename> typename, size_t> typename F, typename TL, typename IL> struct transformN_T_impl;
        template<template<template<typename> typename, size_t> typename F, 
                 template<template<typename> typename...> typename TL,
                 template<typename> typename... I, 
                 size_t... IN>
        struct transformN_T_impl<F, TL<I...>, std::index_sequence<IN...>> {
            static_assert(sizeof...(I) == sizeof...(IN));
            typedef Meta::List<typename F<I, IN>::type...> type;                
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

        template<typename L> struct middle_impl;
        template<template<typename...> typename L, typename... I>
        struct middle_impl<L<I...>> {
            static inline constexpr size_t mid = sizeof...(I) / 2;
            using type = typename nth_element_impl<mid, L<I...>>::type;
        };
        
        
        template<typename List, typename T> struct count_impl {
            template<typename U>
            using p = std::is_same<U, T>;
            using filtered = typename filter_impl<p, List>::type;
            inline static constexpr size_t value = filtered::size;
        };

        template<typename L, template<typename>typename T> struct count_T_impl;
        template<template<template<typename>typename...> typename L, 
                 template<typename>typename F, 
                 template<typename>typename T, 
                 template<typename>typename... R> 
        struct count_T_impl<L<F, R...>, T> {
            inline static constexpr size_t value = [] {
                constexpr auto v = is_same_template_v<T, F>;
                if constexpr(sizeof...(R) > 0) {
                    return count_T_impl<L<R...>, T>::value + v;
                }
                else {
                    return v;
                }
            }();
        };
        
        template<typename L, typename T, size_t N> struct index_impl;
        template<template<typename...> typename L, typename F, typename... R, typename T, size_t N>
        struct index_impl<L<F, R...>, T, N> {
            inline static constexpr size_t value = [] {
                if constexpr(std::is_same<F, T>::value) {
                    return N;
                }
                else {
                    return index_impl<L<R...>, T, N+1>::value;
                }
            }(); 
        };
        
        template<typename L, template<typename> typename T, size_t N> struct index_T_impl;
        template<template<template<typename>typename...> typename L, 
                 template<typename> typename F, 
                 template<typename> typename... R, 
                 template<typename> typename T, size_t N>
        struct index_T_impl<L<F, R...>, T, N> {
            inline static constexpr size_t value = [] {
                if constexpr(is_same_template_v<F, T>) {
                    return N;
                }
                else {
                    return index_T_impl<L<R...>, T, N+1>::value;
                }
            }(); 
        };
        
        template<typename List, typename T> struct contains_impl {
            inline static constexpr bool value = (count_impl<List, T>::value > 0);
        };
        
        template<typename List, template<typename>typename T> struct contains_T_impl {
            inline static constexpr bool value = (count_T_impl<List, T>::value > 0);
        };

        template<typename, typename> struct concat_impl;
        template<template<typename...> typename L1, typename... I1, template<typename...> typename L2, typename... I2> 
        struct concat_impl<L1<I1...>,  L2<I2...>> {
            typedef L1<I1..., I2...> type;
        };
        
        template<typename> struct is_set_impl;
        template<template<typename, typename...> typename L, typename F, typename... I>
        struct is_set_impl<L<F, I...>> {
            using type = typename std::conditional<contains_impl<L<I...>, F>::value, 
                                                  std::integral_constant<bool, false>, 
                                                  typename is_set_impl<L<I...>>::type>::type;
        };
        template<template<typename> typename L, typename I>
        struct is_set_impl<L<I>> {
            using type = std::integral_constant<bool, true>;
        };
        
        template<typename> struct is_set_T_impl;
        template<template<template<typename>typename...> typename L, template<typename> typename F, template<typename> typename ... I>
        struct is_set_T_impl<L<F, I...>> {
            using type = typename std::conditional<contains_T_impl<L<I...>, F>::value, 
                                                  std::integral_constant<bool, false>, 
                                                  typename is_set_T_impl<L<I...>>::type>::type;
        };
        template<template<template<typename>typename...> typename L, template<typename>typename I>
        struct is_set_T_impl<L<I>> {
            using type = std::integral_constant<bool, true>;
        };

        template<typename, typename> struct contains_all_impl {};
        template<template<typename...> typename L1, typename... I1, template<typename...> typename L2, typename... I2>
        struct contains_all_impl<L1<I1...>, L2<I2...>> {
            using type = std::integral_constant<bool, (contains_impl<L1<I1...>, I2>::value && ... && true)>;
        };
        
        template<typename, typename> struct all_same_impl;
        template<typename T, template<typename...> typename L, typename... I>
        struct all_same_impl<T, L<I...>> {
            static inline constexpr bool value = (std::is_same<T, I>::value && ... && true);
        };

        template<typename L, auto N> struct partial_sum_impl;
        template<typename... C, template<typename...>typename L, typename F, auto N>
        struct partial_sum_impl<L<F, C...>, N> {
            static inline constexpr size_t V = F::value + N;
            typedef typename concat_impl<Meta::List<std::integral_constant<size_t, V>>, typename partial_sum_impl<L<C...>, V>::type>::type  type;
        };
        template<template<typename>typename L, typename F, auto N>
        struct partial_sum_impl<L<F>, N> {
            typedef Meta::List<std::integral_constant<size_t, F::value + N>> type;
        };
        
        template<typename L> struct reverse_impl;
        template<template<typename...> typename L, typename F, typename... II>
        struct reverse_impl<L<F, II...>> {
            typedef typename concat_impl<typename reverse_impl<L<II...>>::type, L<F>>::type type;
        };
        template<template<typename...> typename L, typename F>
        struct reverse_impl<L<F>> {
            typedef L<F> type;
        };
        template<concepts::List List> 
        struct visitBinary {
            template<typename I, typename C>
            inline static constexpr void at(I index, const C& callable) {
                constexpr I mid = size_v<List> / 2;
                using mid_t = middle_element<List>;
                if (index == mid) {
                    callable(Wrapper<mid_t>{});
                }
                else if (index < mid) {
//                    using lower_t = lower_half<List>;
//                    visitBinary<lower_t>::at(mid - index, callable);
                }
                else {
//                    using upper_t = upper_half<List>;
//                    visitBinary<upper_t>::at(index - mid, callable);                    
                }
            }
        };
        
        struct visitJT {
            template<typename> struct vhelper;
            template<template<auto> typename A, auto I>
            struct vhelper<A<I>> {
                inline static constexpr auto index = I;
            };
            
            template<concepts::List List, typename I, typename C>
            inline static constexpr void at(const I index, const C& callable) {
                if constexpr(size_v<List> > 0) {
                    using first = Meta::front<List>;
                    if (index == vhelper<first>::index) {
                        callable(Wrapper<first>{});
                    }
                    else {
                        using r_t = Meta::rest<List>;
                        visitJT::template at<r_t>(index, callable);
                    }                    
                }
            }
        };
        
        template<concepts::List List> 
        struct visit {
            using first = Meta::front<List>;
            template<typename I, typename C>
            inline static constexpr void at(const I index, const C& callable) {
                if (index == I{0}) {
                    if constexpr(!std::is_same_v<first, void>) {
                        callable(Wrapper<first>{});
                    }
                }
                else {
                    visit<Meta::rest<List>>::at(I(index - 1), callable);
                }
            }
            template<auto... II, typename C>
            inline static void constexpr all(std::index_sequence<II...>, const C& callable) {
                (at(II, callable),...);
            }

            template<typename C>
            inline static size_type<List> find(const C& callable, const size_type<List>& index) {
                if (callable(Wrapper<first>{})) {
                    return index;
                }
                else {
                    return visit<Meta::rest<List>>::find(callable, index + 1);
                }
            }            
        };
        template<> 
        struct visitBinary<Meta::List<>> {
            template<typename I, typename C>
            constexpr inline static void at(I, const C&) {
//                assert(false);
            }
        };
        template<> 
        struct visit<Meta::List<>> {
            template<typename I, typename C>
            constexpr inline static void at(I, const C&) {
//                assert(false);
            }
            template<typename C>
            inline static void constexpr all(std::index_sequence<>, const C&) {}
            
            template<typename C, typename N>
            constexpr inline static N find(const C&, N) {
                return std::numeric_limits<N>::max();
            }
        };
        template<typename L>
        struct unique_impl;
        template<template<typename...> typename L, typename F, typename... II>
        struct unique_impl<L<F, II...>> {
            typedef typename std::conditional<Meta::contains<L<II...>, F>::value, 
                                              typename unique_impl<L<II...>>::type, 
                                              Meta::concat<L<F>, 
                                                           typename unique_impl<L<II...>>::type>
                                             >::type type;
        };
        template<template<typename...> typename L, typename F>
        struct unique_impl<L<F>> {
            typedef L<F> type;
        };
        
        template<auto N, typename T>
        struct NumberedNode {
            inline static constexpr size_t index = N;
            typedef T type;
        };
    
        template<typename L, typename I> struct make_numbered;
        template<template<typename...> typename L, typename... TT, auto... II>
        struct make_numbered<L<TT...>, std::index_sequence<II...>> {
            typedef L<NumberedNode<II, TT>...> type;
        };
        
        template<typename Key, typename Map>
        struct map_impl;
        template<typename Key, template<typename...>typename L, typename... PP, typename K, typename V>
        struct map_impl<Key, L<L<K,V>, PP...>> {
            typedef std::conditional_t<std::is_same_v<K, Key>, V, typename map_impl<Key, L<PP...>>::type> type;
        };
        template<typename Key, template<typename...>typename L>
        struct map_impl<Key, L<>> {
            typedef void type;
        };
        
        template<typename L>
        struct value_or_impl;
        template<typename... T>
        struct value_or_impl<List<T...>> {
            inline static constexpr auto value = (T::value | ...);  
        };
        
    } // !detail
        
    template<template<typename...> typename F, concepts::List List>
    using apply = typename detail::apply_impl<F, List>::type;
    
    template<concepts::List List, typename T>
    using push_front = typename detail::push_front_impl<List, T>::type;
 
    template<concepts::List List, typename T>
    using push_back = typename detail::push_back_impl<List, T>::type;

    template<concepts::List List>
    using back = typename detail::back_impl<List>::type;
    
    template<typename> struct tfront;
    template<template<typename> typename F,
             template<template<typename> typename...> typename TL,
             template<typename> typename... R>
    struct tfront<TL<F, R...>> {
        template<typename T> using type = F<T>;
    };
    
    template<template<typename> typename Func, concepts::List List>
    using transform = typename detail::transform_impl<Func, List>::type;

    template<template<typename> typename Func, concepts::List List>
    using transform_type = typename detail::transform_type_impl<Func, List>::type;

    template<template<typename> typename Func, typename List>
    using transform_list = typename detail::transform_type_impl<Func, List>::type;
    
    template<template<typename, size_t> typename Func, concepts::List List>
    using transformN = typename detail::transformN_impl<Func, List, std::make_index_sequence<size<List>::value>>::type;

    template<template<typename, typename> typename Func, concepts::List List1, concepts::List List2>
    using transform2 = typename detail::transform2_impl<Func, List1, List2>::type;
    
    template<template<template<typename> typename> typename F, typename TL> struct transform_T;
    template<template<template<typename> typename> typename F, 
             template<template<typename> typename...> typename TL,
             template<typename> typename... I>
    struct transform_T<F, TL<I...>> {
        typedef Meta::List<typename F<I>::type...> type;                
    };
    
    template<template<template<typename> typename, size_t> typename F, typename TL> struct transformN_T;
    template<template<template<typename> typename, size_t> typename F, 
             template<template<typename> typename...> typename TL,
             template<typename> typename... I>
    struct transformN_T<F, TL<I...>> {
        typedef typename detail::transformN_T_impl<F, TL<I...>, std::make_index_sequence<sizeof...(I)> >::type type;                
    };
    
    template<template<typename> typename Pred, concepts::List List>
    using filter = typename detail::filter_impl<Pred, List>::type;
    
    template<size_t N, concepts::List List>
    using nth_element = typename detail::nth_element_impl<N, List>::type;

    template<concepts::List List>
    using reverse = typename detail::reverse_impl<List>::type;
    
    template<concepts::List List, typename... T>
    struct containsAll : public std::integral_constant<bool, detail::contains_all_impl<List, Meta::List<T...>>::type::value> {};

    template<concepts::List List, typename... T>
    inline static constexpr bool containsAll_v = containsAll<List, T...>::value;
    
    template<concepts::List List, typename T>
    struct count: public std::integral_constant<size_t, detail::count_impl<List, T>::value> {};

    template<concepts::List List, typename T>
    struct index: public std::integral_constant<size_t, detail::index_impl<List, T, 0>::value> {};

    template<concepts::List List, typename T>
    inline static constexpr auto index_v = index<List, T>::value;
    
    template<typename List, template<typename> typename T>
    struct index_T: public std::integral_constant<size_t, detail::index_T_impl<List, T, 0>::value> {};
    
    template<concepts::List List>
    struct is_set : public std::integral_constant<bool, detail::is_set_impl<List>::type::value> {};

    template<concepts::List List>
    inline static constexpr bool is_set_v = is_set<List>::value;
    
    template<typename TL>
    struct is_set_T : public std::integral_constant<bool, detail::is_set_T_impl<TL>::type::value> {};

    template<typename T, concepts::List L>
    struct all_same : public std::integral_constant<bool, detail::all_same_impl<T, L>::value> {};

    template<typename T, concepts::List L>
    inline static constexpr bool all_same_v = Meta::all_same<T, L>::value;
    
    template<concepts::List L>
    struct all_same_front : public std::integral_constant<bool, detail::all_same_impl<front<L>, L>::value> {};
    
    template<concepts::List L>
    inline static constexpr bool all_same_front_v = Meta::all_same_front<L>::value;
    
    template<concepts::List List>
    using pop_front = rest<List>;
    
    template<concepts::List List>
    using pop_back = reverse<rest<reverse<List>>>;
    
    template<typename L>
    using partial_sum = typename detail::partial_sum_impl<L, 0>::type;
    
    template<concepts::List List>
    using value_or = typename detail::value_or_impl<List>;
    
    template<concepts::List List>
    inline static constexpr auto value_or_v = detail::value_or_impl<List>::value;
    
    template<concepts::List List, typename I, typename C>
    inline void constexpr visitAt(const I index, const C& callable) {
        detail::visit<List>::at(index, callable);
    }
    template<concepts::List List, typename I, typename C>
    inline void constexpr visitAtJT(const I index, const C& callable) {
        detail::visitJT::template at<List>(index, callable);
    }
    template<concepts::List List, typename I, typename C>
    inline void constexpr visitAtBinary(const I index, const C& callable) {
        detail::visitBinary<List>::at(index, callable);
    }

    template<concepts::List List, typename C>
    inline void constexpr visit(const C& callable) {
        detail::visit<List>::all(std::make_index_sequence<size<List>::value>{}, callable);
    }

    template<concepts::List List, typename C>
    inline size_type<List> find(const C& callable) {
        return detail::visit<List>::find(callable, size_type<List>{0});
    }
    
    template<concepts::List L>
    using unique = typename detail::unique_impl<L>::type;

    template<typename Key, typename Map>
    using map = typename detail::map_impl<Key, Map>::type;
    
    template<typename... T>
    struct always_false : std::false_type {};
    
    template<typename... T>
    inline static constexpr bool always_false_v = always_false<T...>::value;
    
    template<typename T>
    using value_type_of = typename T::value_type;
    
    template<concepts::List List>
    using make_numbered = typename detail::make_numbered<List, std::make_index_sequence<size_v<List>>>::type;    
    
    template<typename T>
    struct nonVoid : public std::true_type {};    
    template<>
    struct nonVoid<void> : public std::false_type {};    

    template<typename T>
    struct isVoid : public std::false_type {};    
    template<>
    struct isVoid<void> : public std::true_type {};    
    
    namespace tests {
        struct A {};
        struct B {};
        struct C {};
        struct D {};
        struct E {};
        
        // list
        using l1 = Meta::List<A, B>;
        static_assert(l1::size == 2);
        // size
        static_assert(size<l1>::value == 2);
        // length
        static_assert(length<A, B>::value == 2);
        // apply
        template<typename... T> struct NewList{typedef NewList<T...> type;};
        using nl1 = apply<NewList, l1>;
        // push_front
        using l2 = Meta::push_front<l1, C>;
        static_assert(l2::size == 3);
        static_assert(index<l2, B>::value == 2);
        // front
        using x1 = Meta::front<l2>;
        static_assert(std::is_same<x1, C>::value);
        
        template<typename T> using add_pointer = T*;
        using l3 = Meta::transform<add_pointer, l2>;
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
        
        using l20 = Meta::List<A, B>;
        using l21 = Meta::List<C, D>;
        using l22 = Meta::transform2<std::pair, l20, l21>;
        static_assert(size<l22>::value == 2);
        using r20 = Meta::front<l22>;
        static_assert(std::is_same<r20::first_type, A>::value);
        static_assert(std::is_same<r20::second_type, C>::value);
        using r21 = Meta::nth_element<1, l22>;
        static_assert(std::is_same<r21::first_type, B>::value);
        static_assert(std::is_same<r21::second_type, D>::value);
        
        template<typename T>
        concept /*bool*/ ConceptisA = std::is_same<T, A>::value;

        template<typename T>
        struct isA : public std::false_type {};
        template<ConceptisA T>
        struct isA<T> : public std::true_type {};
        
        using l100 = Meta::filter<isA, Meta::List<A, A, B, C>>;
        static_assert(Meta::size<l100>::value == 2);
        
        template<typename T, size_t N>
        struct f2 {};
        using l200 = Meta::transformN<f2, Meta::List<A, A, B, C>>;
        static_assert(std::is_same<Meta::front<l200>, f2<A, 0>>::value);        
        static_assert(std::is_same<Meta::nth_element<1, l200>, f2<A, 1>>::value);        
        static_assert(std::is_same<Meta::nth_element<2, l200>, f2<B, 2>>::value);        

        using l300 = Meta::List<A, B, C>;
        static_assert(Meta::is_set<l300>::value);
        using l301 = Meta::List<A>;
        static_assert(Meta::is_set<l301>::value);
        using l302 = Meta::List<A, A>;
        static_assert(!Meta::is_set<l302>::value);
        using l303 = Meta::List<A, B, C, A>;
        static_assert(!Meta::is_set<l303>::value);
        using l304 = Meta::List<E, A, B, C, A>;
        static_assert(!Meta::is_set<l304>::value);
        
        static_assert(Meta::containsAll<l300, A, B, C>::value);
        static_assert(!Meta::containsAll<l300, E>::value);
        static_assert(!Meta::containsAll<l300, E, A>::value);
        static_assert(!Meta::containsAll<l300, E, D>::value);
        
        static_assert(Meta::count<l300, A>::value == 1);
        static_assert(Meta::count<l304, A>::value == 2);
        
        static_assert(Meta::index<l300, A>::value == 0);
        static_assert(Meta::index<l300, C>::value == 2);
        
        using l400 = Meta::concat<l300, l301>;
        static_assert(Meta::size<l400>::value == 4);
        
        static_assert(Meta::all_same<A, l302>::value);
        static_assert(!Meta::all_same<A, l300>::value);
        
        using l401 = Meta::rest<l303>;
        static_assert(std::is_same<Meta::front<l401>, B>::value);
        static_assert(std::is_same<Meta::back<l401>, A>::value);
        
        using l500 = Meta::List<std::integral_constant<uint8_t, 1>, std::integral_constant<uint8_t, 2>, std::integral_constant<uint8_t, 5>>;
        using l500ps = Meta::partial_sum<l500>;
        static_assert(nth_element<0, l500ps>::value == 1);
        static_assert(nth_element<1, l500ps>::value == 3);
        static_assert(nth_element<2, l500ps>::value == 8);
        
        using l600 = Meta::List<A, B, C>;
        using l601 = Meta::reverse<l600>;
        static_assert(std::is_same<Meta::front<l601>, C>::value);
        static_assert(std::is_same<Meta::back<l601>, A>::value);
        
//        static_assert(false);
        
    } // !tests
} // !Meta
