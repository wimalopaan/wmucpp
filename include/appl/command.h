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

#include "util/meta.h"

namespace Command {
    namespace detail {
        inline constexpr bool unique(const auto& values) {
            for(const auto& a : values) {
                for(const auto& b: values) {
                    if (&a != &b) {
                        if (a == b) {
                            return false;
                        }
                    }
                }
            }
            return true;
        }
    }
    template<auto v>
    struct Value {
        typedef decltype(v) value_type;
        inline static constexpr auto value = v;
    };

    template<typename Command, auto v>
    struct Option {
        typedef Command command_type;
        typedef decltype(v) value_type;
        inline static constexpr auto value = v;  
    };
    template<typename CList, typename OList>
    struct CommandSet;
    template<template<typename...> typename CList, typename... CC, template<typename...> typename OList, typename... OO>
    struct CommandSet<CList<CC...>, OList<OO...>> {
        typedef typename Meta::front<CList<CC...>>::value_type value_type;
        static_assert(Meta::all_same<value_type, Meta::List<typename CC::value_type...>>::value);
        static_assert(Meta::all_same<value_type, Meta::List<typename OO::value_type...>>::value);
        
        typedef CList<CC...> commands;
        typedef OList<CC...> options;
        inline static constexpr std::array<std::byte, sizeof...(CC)> commandValues = {CC::value ...};
        inline static constexpr std::array<std::byte, sizeof...(OO)> optionsValues = {(OO::command_type::value | OO::value) ...};
        static_assert(detail::unique(commandValues));
        static_assert(detail::unique(optionsValues));

        template<typename C, typename... O>
        requires (Meta::contains<commands, C>::value) &&
        (std::is_same<C, typename O::command_type>::value && ... && true)
        static value_type value(C, O...) {
            return C::value | (O::value | ... | value_type{0});    
        }
        template<typename C, typename... O>
        requires (Meta::contains<commands, C>::value) &&
        (std::is_same<C, typename O::command_type>::value && ... && true)
        static value_type value() {
            return C::value | (O::value | ... | value_type{0});    
        }
    };
}

