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

#include <type_traits>
#include <utility>

#include "meta.h"

struct A{};
struct B{};
struct C{};
struct D{};

struct TypeListBase {
    typedef TypeListBase base_type;
};
template<typename... T>
struct TypeList : public TypeListBase {
    inline static constexpr size_t size = sizeof...(T);
};

template<typename T>
concept bool IsTypeList() {
    return requires {
        typename T::base_type;  
    } && std::is_same<typename T::base_type, TypeListBase>::value;
}



template<typename, typename> 
struct AddFront_Impl;
template<typename  T, typename ...Args>
struct AddFront_Impl<T, TypeList<Args...>>{
    typedef TypeList<T, Args...> type;
};
template<typename  T, typename ...Args>
using AddFront = typename AddFront_Impl<T, Args...>::type;

template<template<typename> typename Pred, typename... Tail> 
struct Filter_Impl;
template<template<typename> typename Pred> 
struct Filter_Impl<Pred> { 
    typedef TypeList<> type; 
};
template <template<typename> typename Pred, typename Head, typename... Tail>
struct Filter_Impl<Pred, Head, Tail...> {
    typedef typename std::conditional<Pred<Head>::value,
                               AddFront<Head, typename Filter_Impl<Pred, Tail...>::type>,
                               typename Filter_Impl<Pred, Tail...>::type
                          >::type type;
};
template <template<typename> typename Pred, typename Head, typename... Tail>
using Filter = typename Filter_Impl<Pred, Head, Tail...>::type;



template <template<typename> typename Pred, typename Head, typename ...Tail>
using FilteredList = typename Filter_Impl<Pred, Head, Tail...>::type;

template<typename...>
struct Head_impl;
template<typename F, typename... Tail>
struct Head_impl<F, Tail...> {
    typedef F type;
};
template<typename... Tail>
using Head = typename Head_impl<Tail...>::type;

template<typename...>
struct ListHead_impl;
template<typename... T>
struct ListHead_impl<TypeList<T...>> {
    typedef Head<T...> type;
};
template<IsTypeList T>
using ListHead = typename ListHead_impl<T>::type;

template<template<typename> typename Pred, typename... T>
struct ListFilter_impl;
template<template<typename> typename Pred>
struct ListFilter_impl<Pred> {
    typedef TypeList<> type;
};
template <template<typename> typename Pred, typename... T>
struct ListFilter_impl<Pred, TypeList<T...>> {
    typedef typename Filter_Impl<Pred, T...>::type type;
};
template<template<typename> typename Pred, IsTypeList List>
using ListFilter = typename ListFilter_impl<Pred, List>::type;

template<typename T>
struct Predicate1 {
    inline static constexpr bool value = std::is_same<T, A>::value || std::is_same<T, B>::value;
};
template<typename T>
struct Predicate2 {
    inline static constexpr bool value = std::is_same<T, D>::value ;
};

template<typename... T>
struct ListCat_impl;
template<>
struct ListCat_impl<> {
    typedef TypeList<> type;
};
template<typename... T1, typename... T2>
struct ListCat_impl<TypeList<T1...>, TypeList<T2...>> {
    typedef TypeList<T1..., T2...> type;
};
template<IsTypeList TU1, IsTypeList TU2>
using ListCat = typename ListCat_impl<TU1, TU2>::type;

int main() {
    using fl1 = FilteredList<Predicate1, A, B, C, D>;
    using fl2 = FilteredList<Predicate2, A, B, C, D>;
    
    using tup1 = ListFilter<Predicate1, TypeList<A, B, C, D>>;
    
    static_assert(std::is_same<tup1, fl1>::value);
    
    using x = ListHead<TypeList<A, B>>;
    static_assert(std::is_same<x, A>::value);    
    
    static_assert(fl1::size == 2);
    static_assert(fl2::size== 1);
    
    using fl3 = ListCat<fl1, fl2>;
    static_assert(fl3::size == 3);
}
