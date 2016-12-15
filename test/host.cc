/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <iostream>
#include <iterator>
//#include "../tests/algorithm.h"
////#include "../tests/percent.h"
////#include "../tests/duration.h"
////#include "../tests/physical.h"
//#include "../tests/types.h"
//#include "../tests/atomic.h"

//#include <math.h>
//#include "util/fixedpoint.h"

#include <numeric>
#include <array>
#include <utility>

constexpr int foo(size_t v) {
    return v + 0x30;
}

template<typename F, size_t... II>
constexpr auto createArray(F f, std::index_sequence<II...>) {
    return std::array<int, sizeof...(II)> {{f(II)...,}};
}
        
int main()
{
    constexpr auto a = createArray(foo, std::make_index_sequence<10>{});
//    constexpr auto b = createArray([](size_t){return 1;}, std::make_index_sequence<10>{});
    
    std::copy(std::begin(a), std::end(a), std::ostream_iterator<double>(std::cout, ", "));
    
//    constexpr auto a = createArray2(foo, 1, 2);
    
    
//    constexpr FixedPoint<int16_t, 4> f1 = 1.75_fp;          
//    constexpr FixedPoint<int16_t, 4> f2(0.5);          
//    constexpr FixedPoint<int16_t, 4> f3(-0.5);
    
//    std::cout << f3.raw() << std::endl;
//    std::cout << f3.integer() << std::endl;
//    std::cout << f3.fractional() << std::endl;

//    constexpr FixedPoint<int16_t, 4> f4(-1.5);
//    std::cout << f4.integer() << std::endl;
//    std::cout << f4.fractional() << std::endl;
    
}

void assertFunction(bool b, const char *function, const char *file, unsigned int line) {
    if (!b) {
        std::cout << "Assertion failed in: " << function << " file: " << file << " line: " << line << std::endl;
    }
}

