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
#include "std/array"
#include "std/utility"


constexpr uint8_t foo(size_t v) {
    return v + 0x30;
}

template<typename ItemType, typename F, size_t... II>
constexpr auto createArray(F f, std::index_sequence<II...>) {
    return std::array<ItemType, sizeof...(II)> {{f(II)...}};
}

int main()
{
    constexpr int Size = 10;
    std::array<uint8_t, Size> values = {1, 1, 1};
    
    constexpr auto indices = createArray<uint8_t>([](size_t v){return v + 1;}, std::make_index_sequence<3>{});
//    constexpr auto indices = []<typename T, typename F, size_t... II>(T t, F f, std::index_sequence<II...>){std::array<T, sizeof...(II)> {{f(II)...}}}(int(0), [](size_t v){return v + 1;}, std::make_index_sequence<3>{});

//    auto x = []<typename T>(T p){return p;};
    
    uint8_t sum = 0;
    for(const auto& i : indices) {
        sum += values[i];
    }
    return sum;    
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) {
//    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif

