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

//#define MEM
//#define NDEBUG

#include "rcsensorled01.h"
#include "console.h"
#include "util/meta.h"

namespace {
    constexpr bool useTerminal = true;
}

using terminalDevice = std::conditional<useTerminal, SSpi0, void>::type;
using terminal = std::basic_ostream<terminalDevice>;

template<typename... T>
struct Distributor {
    using Items = Meta::filter<Meta::nonVoid, Meta::List<T...>>;
    template<typename U> struct NonVoidDistributor;
    template<template<typename...> typename L, typename... U>
    struct NonVoidDistributor<L<U...>> {
        inline static void init() {
            (U::init(), ...);
        }
    };
    inline static void init() {
        NonVoidDistributor<Items>::init();
    }
};

//using isrRegistrar = IsrRegistrar<rpm>;
using isrRegistrar = IsrRegistrar<>;
using distributor = Distributor<terminalDevice, isrRegistrar, rpm>;

int main() {
    uint32_t counter = 0;
    distributor::init();
    
    std::outl<terminal>("Test04"_pgm);

    {
        Scoped<EnableInterrupt<>> ei;
        while(true) {
            if (counter > 900000) {
                counter = 0;
                const auto upm = rpm::rpm();
                rpm::reset();
                if (upm) {
                    std::outl<terminal>("RPM: "_pgm, upm.value());   
                }
                else {
                    std::outl<terminal>("--- "_pgm);   
                }
            }  
            rpm::periodic();
            
// alte Variante mit PinChangeInterrupt
//            Util::delay(500_ms);
//            const auto upm = rpm::rpm();
//            rpm::reset();
//            if (upm) {
//                std::outl<terminal>("RPM: "_pgm, upm.value());   
//            }
//            else {
//                std::outl<terminal>("--- "_pgm);   
//            }
            ++counter;
        }
    }
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    while(true) {}
}
#endif

// RPM
//ISR(PCINT1_vect) {
//    isrRegistrar::isr<AVR::ISR::PcInt<1>>();
//}
