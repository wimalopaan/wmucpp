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

#include <cstdint>
#include <cstddef>

#include "flag.h"

namespace Hal {
    template<typename FR, template<typename> typename X, size_t v>
    struct MakeNumberedFlag {
        typedef X<Hal::Flag<FR, v>> type;  
    };
    
    template<typename FlagRegister, template<typename> typename ... X>
    struct Controller {
        static_assert(sizeof...(X) <= 8, "too much ressources");
        using ressourceList = Meta::TList<X...>;
        static_assert(Meta::is_set_T<ressourceList>::value, "all ressources must be different");
        
        template<template<typename> typename T, size_t v>
        using makeFlags = MakeNumberedFlag<FlagRegister, T, v>;
        
        using numberedRessouceList = typename Meta::transformN_T<makeFlags, ressourceList>::type;
    
        template<template<typename> typename T>
        struct get {
            static constexpr auto index = Meta::index_T<ressourceList, T>::value;
            typedef Meta::nth_element<index, numberedRessouceList> type;
        };
    };
}