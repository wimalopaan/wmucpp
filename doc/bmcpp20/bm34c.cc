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

#define NDEBUG

#include "mcu/avr8.h"
#include "units/percent.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

using namespace std::literals::quantity;

volatile auto y = [](){
    std::array<uint8_t, 10> a;
    std::iota(std::begin(a), std::end(a), 1);
    return a;
}();

volatile std::percent z = 0_ppc;

template<uint8_t Begin, uint8_t End, typename T>
struct Generator {
    static void generate(T value) {
        z = std::fastScale<0, Begin>(value);
        if constexpr(Begin < End) {
            Generator<Begin + 1, End, uint8_t>::generate(value);
        }
    }
};

int main() {
    for(auto x : y) {
        Generator<101, 255, uint8_t>::generate(x);    
    }
    while(true) {}
}

template<typename L>
void assertFunction(const PgmStringView&, const PgmStringView&, L) noexcept {
    while(true) {}
}
