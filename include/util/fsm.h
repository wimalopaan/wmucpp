/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "std/array.h"

namespace Util {

struct Event {
    const int value = 0;
};

namespace detail {
typedef int(*nextFunction_t)(const Event&, int) ;
}

template<typename... SS>
class Fsm {
    template<typename S1, typename... S2> friend Fsm<S1, S2...>& create();
public:
    void process(const Event& e) {
        state = l[state](e, state);
    }
private:
    static detail::nextFunction_t l[sizeof...(SS)];
    int state = 0;
};

template<typename... SS>
detail::nextFunction_t Fsm<SS...>::l[sizeof...(SS)] {};

template<int N>
void createA(detail::nextFunction_t*) {
}

template<int N, typename S1, typename...S2>
void createA(detail::nextFunction_t* p)
{
    p[N] = S1::next;
    createA<N + 1, S2...>(p);
}

template<typename S1, typename...S2>
Fsm<S1, S2...>& create() {
    static Fsm<S1, S2...> fsm;
    createA<0, S1, S2...>(Fsm<S1, S2...>::l);
    return fsm;
}

}
