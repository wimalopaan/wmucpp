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

#include "std/traits.h"
#include "std/byte.h"

namespace MCU {

template<typename T>
concept bool Timer() {
    return std::is_same<T, void>::value || requires(T t) {
        T::mcuTimer();
        T::mcuInterrupts();
        T::template prescale<1>();
    };
}

template<typename P>
concept bool Port() { 
    return requires (P p) { 
        p.template set<std::byte{0}>();
        p.set(std::byte{0});
        p.get();
    };
}

template<typename S>
concept bool Stream() { 
    return requires (S s) { 
        typename S::device_type;
        typename S::line_terminator_type;
    };
}

template<typename P>
concept bool Pin() { 
    return std::is_same<P, void>::value || requires (P p) { 
        p.on();
        p.off();
    };
}

template<typename I>
concept bool Interrupt() { 
    return std::is_same<I, void>::value || requires (I i) {
        I::number;
    };
}

template<typename I>
concept bool IServiceR() { 
    return std::is_same<I, void>::value || requires (I i) {
        I::isr();
        I::isr_number;
    };
}

template<typename L>
concept bool Letter() {
    return requires(L l) {
        L::letter;
    };
}

template<typename C>
concept bool SingleComponent() {
    return requires(C c) {
        C::address;
    };
}

template<typename C>
concept bool MultipleComponent() {
    return requires(C c) {
        C::count;
//        C::template Address<0>::value; // not possible beacuse of timer numbering scheme 
    };
}


}
