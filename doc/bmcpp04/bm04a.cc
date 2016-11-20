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

template<int N>
void ttt() {}

constexpr int bar(const int v) {
    if (v > 0){
        return v * 2;
    }
    return 2;
}

constexpr int foo(const int v) {
//    constexpr auto x =  bar(v);
//    ttt<x>();
    return v;
}

int main() {
    constexpr auto a = foo(1);
    constexpr auto b = bar(1);
    ttt<b>();
}
