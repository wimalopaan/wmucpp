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

#include "std/traits.h"

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
        p.template set<0>();
        p.set(0);
        p.get();
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

}
