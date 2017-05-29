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

#include <stdint.h>
#include "std/traits.h"
#include "compiler/compiler.h"

namespace MCU {
    struct Class8Bit {};
    struct Class16Bit {};
    struct Class32Bit {};
    
    template<typename T, typename C>
    struct is_register_type_base {
        typedef T type;
        typedef C compiler_type;
        inline static constexpr bool value = false;
    };
    template<>
    struct is_register_type_base<uint8_t, Compiler::Gcc> {
        typedef uint8_t type;
        typedef Compiler::Gcc compiler_type;
        inline static constexpr bool value = true;
    };
    
    
    template<typename T, typename Compiler>
    struct is_register_type : public is_register_type_base<typename std::remove_cv<T>::type, Compiler> {};
    

}