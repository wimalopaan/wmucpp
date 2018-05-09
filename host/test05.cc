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
#include <array>
#include <iostream>

constexpr size_t maxPrime(size_t maximum) {
    size_t mp = 0;
    for(size_t v = 2; v <= maximum; ++v) {
        bool vIsPrime = true;
        for(size_t d = 2; d < v/2; ++d) {
            if ((v % d) == 0) {
                vIsPrime = false;
                break;
            }
        }
        if (vIsPrime) {
            mp = v;
        }
    }
    return mp;
}

template<size_t Maximum>
constexpr size_t maxPrime2() {
    std::array<bool, Maximum> test{};
    for(size_t i = 2; i < Maximum / 2; ++i) {
        for(size_t f = 2; f < Maximum / 2; ++f) {
            size_t np = i * f;
            if (np < Maximum) {
                test[np] = true;
            }
            else {
                break;
            }
        }
    }
    for(size_t m = Maximum - 1; m >= 2; --m) {
        if (!test[m]) {
            return m;
        }
    }
    return 0;
}

int main() {
//    constexpr auto mp = maxPrime(100000);   
    constexpr auto mp = maxPrime2<100000>();   
    std::cout << "MP: " << mp << '\0';    
}
