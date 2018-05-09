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

#include <cstdint>
#include <cstddef>

#include "mcu/avr8.h"
#include "mcu/register.h"

#include "util/meta.h"

template<typename T, typename F>
struct A {
    A() {
        mFlag.set();
    }
    T mData;
    F mFlag;
};

template<typename F, typename T>
struct B {
    B() {
        if (mFlag) {
            mFlag.reset();
        }
    }
    T mData;
    F mFlag;
};

template<typename F>
using AF = A<uint8_t, F>;

template<typename F>
using BF = B<F, uint16_t>;

using fr0 = AVR::RegisterFlags<typename DefaultMcuType::GPIOR, 0, std::byte>;

template<typename FR, uint8_t Bit>
struct Flag {
    static constexpr std::byte mask = std::byte{1 << Bit};
    void set() {
        FR::get() |= mask;
    }  
    void reset() {
        FR::get() &= ~mask;
    }  
    explicit operator bool() {
        return std::any(FR::get() & mask);                  
    }
};

    template<typename FR, template<typename> typename... U>
    struct Controller {
        using tlist = Meta::TList<U...>;

        template<typename T>
        using f = typename Meta::tfront<tlist>::template type<T>;
        
        template<template<typename> typename X, auto N>
        struct MakeNumbered {
            typedef X<Flag<FR,N>> type;
        };

        template<template<typename> typename X>
        struct MakeNumberedFix {
            typedef X<Flag<FR,0>> type;
        };
        
//        typedef Meta::transform_T<MakeNumberedFix, tlist> t2;

        
        
        template<template<typename...> typename T>
        struct get {
            typedef T<Flag<FR,0>> type;
        };
    };
    
    
    using contr = Controller<fr0, AF, BF>;
    
    using AA2 = contr::get<AF>::type;
    
int main() {
    AA2 a;
    while(true) {
    }
}