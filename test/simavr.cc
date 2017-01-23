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

#include "console.h"
#include "simavr/simavrdebugconsole.h"

using terminal = SimAVRDebugConsole;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

#include "../tests/duration.h"
#include "../tests/algorithm.h"
#include "../tests/percent.h"
#include "../tests/physical.h"
#include "../tests/types.h"
#include "../tests/atomic.h"

int main(){
//    std::cout << -1 << std::endl;
    
    
    constexpr FixedPoint<int16_t, 4> f = 1.75_fp;
    std::cout << f << std::endl;    

    constexpr FixedPoint<int16_t, 4> f2;
    std::cout << f2 << std::endl;    

    constexpr FixedPoint<uint16_t, 4> f3(1.3);
    std::cout << f3 << std::endl;    
    
    constexpr FixedPoint<int16_t, 4> f4(-1.25);
    std::cout << f4 << std::endl;    

    constexpr FixedPoint<int16_t, 4> f5(-0.1);
    std::cout << f5 << std::endl;    

    constexpr Fraction<uint8_t> fr1{1};
    std::cout << fr1 << std::endl;

    std::array<uint8_t, 8> b;
    b[0] = 1;
    std::cout << std::crc8(b) << std::endl;
    
    
    while(true);
}
