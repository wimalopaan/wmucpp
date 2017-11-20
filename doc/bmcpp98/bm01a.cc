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

#include "std/type_traits"

struct A {};
struct B {};
struct C {};

template<typename T>
concept bool AorB() {
    return std::is_same<T, A>::value || std::is_same<T, B>::value;
}

//template<typename T>
//concept bool NotAorB() {
//    return !(std::is_same<T, A>::value || std::is_same<T, B>::value);
//}

template<typename T>
class X {};

template<AorB T>
class X<T> {};

int main() {
    X<A> x1;
    X<B> x2;
    X<C> x3;
}
