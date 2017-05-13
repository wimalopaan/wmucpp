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

#include <stdint.h>
#include "simavr/simavrdebugconsole.h"

volatile uint8_t a = 0;
volatile uint8_t b = 0;

volatile uint8_t x = 1;

struct A {
    static constexpr const uint8_t value = 1;
    static void init() {
        a = 1;
        SimAVRDebugConsole::put(std::byte{'a'});
    }
};
struct B {
    static constexpr const uint8_t value = 1;
    static void init() {
        b = 2;
        SimAVRDebugConsole::put(std::byte{'b'});
    }
};

template<typename... PP>
struct ForEachCheckN {
    template<int N, typename P, typename... PPP>
    struct F {
        static void init(int i) {
            if (i == P::value) {
                P::init();
            }
            F<N - 1, PPP..., void>::init(i);
        }
    };
    template<typename P, typename... PPP>
    struct F<0, P, PPP...> {
        static void init(int) {}
    };

    static void init(int i) {
        F<sizeof...(PP), PP...>::init(i);
    }
};

template<typename... PP>
struct ForEachCheck {
    template<typename P, typename... PPP>
    struct F {
        static void init(int i) {
            if (i == P::value) {
                P::init();
            }
            F<PPP..., void>::init(i);
        }
    };
    template<typename... PPP>
    struct F<void, PPP...> {
        static void init(int) {}
    };

    static void init(int i) {
        F<PP...>::init(i);
    }
};

int main()
{
    ForEachCheck<A, B>::init(x);

    SimAVRDebugConsole::put(std::byte{'\r'});
    SimAVRDebugConsole::put(std::byte{'\n'});

    while(true);

}
