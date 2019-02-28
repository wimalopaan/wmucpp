/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2019 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <mcu/avr.h>
#include <cstdint>
#include <etl/stringbuffer.h>
#include <etl/span.h>

//template<typename T>
//struct S {
//    using type = T;
//};

//volatile S<int> s;

//template<typename T>
//void f(T&) {
//    using n = typename T::type;
    
////    S<n>::_;
//}

//int main() {
//     f(s);
//}


volatile etl::StringBuffer<10> b;

int main() {
//    b[0] = etl::Char{' '};
//    etl::make_span<0, 1>(b).insertLeftFill(" "_pgm);
//    etl::make_span<0, 3>(b).insertLeft(etl::Char{' '}, etl::Char{' '});
    
    
//    etl::fill(etl::make_span<0, 3>(b), etl::Char{'a'});
    etl::apply(etl::make_span<0, 3>(b), [](auto& c){c |= etl::Char{0x80};});
}

