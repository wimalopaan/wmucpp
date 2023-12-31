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

#include <stdint.h>
#include <stddef.h>
#include "std/array"


constexpr int foo(size_t v) {
    return v + 0x30;
}

template<typename F, size_t... II>
constexpr auto createArray(F f, std::index_sequence<II...>) {
    std::array<int, sizeof...(II)> a = {{f(II)...,}};
}
        
int main()
{
    [[maybe_unused]] std::array<size_t, 10> c = {1, 2};

}
void assertFunction(bool, const char* , const char *, unsigned int) {
}

