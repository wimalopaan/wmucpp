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

#include <cstdint>
#include <cstddef>

#include "util/meta.h"
#include "flag.h"

namespace Hal {
    template<typename FlagRegister, template<typename> typename ... X>
    requires requires() {
        FlagRegister::get();
        (X<FlagRegister>::number_of_flags,...);
    } 
    struct Controller {
        static_assert(sizeof...(X) <= 8, "too much ressources");
        using ressourceList = Meta::TList<X...>;
        static_assert(Meta::is_set_T<ressourceList>::value, "all ressources must be different");
        
        using numberOfBitsList = Meta::List<std::integral_constant<uint8_t, X<FlagRegister>::number_of_flags>...>;
        
        template<typename L, auto N> struct partial_sum_pairs_impl;
        template<typename... C, template<typename...>typename L, typename F, auto N>
        struct partial_sum_pairs_impl<L<F, C...>, N> {
            inline static constexpr auto Next = N + F::value;
            using x = std::pair<std::integral_constant<uint8_t, N>, std::integral_constant<uint8_t, Next - 1>>;
            using type = typename std::conditional<(sizeof...(C) == 0), Meta::List<x>,
                                          Meta::concat<Meta::List<x>, typename partial_sum_pairs_impl<L<C...>, Next>::type>>::type;
        };
        template<template<typename...>typename L, auto N>
        struct partial_sum_pairs_impl<L<>, N> {
            typedef L<> type;
        };

        template<typename L>
        using partial_sum_pairs = typename partial_sum_pairs_impl<L, 0>::type;
        
        using startEndBitList = partial_sum_pairs<numberOfBitsList>;
    
        template<template<typename>typename T, typename Pair>
        struct makeNumberedFlags {
            typedef T<Hal::Flag<FlagRegister, Pair::first_type::value, Pair::second_type::value>> type;  
        };
        
        template<template<template<typename>typename, typename> typename F, typename L1, typename L2> struct buildNumberedRessources;
        template<template<template<typename>typename, typename> typename F, 
                 template<template<typename>typename...> typename L1, template<typename>typename... I1, 
                 template<typename...> typename L2, typename... I2>
        struct buildNumberedRessources<F, L1<I1...>, L2<I2...>> {
            static_assert(sizeof...(I1) == sizeof...(I2));
            typedef Meta::List<typename F<I1, I2>::type...> type;
        };
        
        using numberedRessouceList = typename buildNumberedRessources<makeNumberedFlags, ressourceList, startEndBitList>::type;
    
        struct detail {
            template<template<typename> typename T>
            struct get {
                static constexpr auto index = Meta::index_T<ressourceList, T>::value;
                typedef Meta::nth_element<index, numberedRessouceList> type;
            };
        };
        template<template<typename> typename T>
        using get = typename detail::template get<T>::type;
    };
}
