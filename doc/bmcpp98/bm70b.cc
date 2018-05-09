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

#include <cstddef>
#include <type_traits>
#include <array>

constexpr auto transform1(const auto& v) {
    return std::integral_constant<int, v>{}; // v ist not constexpr in this context
}

template<typename C, typename T>
concept bool Callable() {
    return requires(C l) {
        {l()} -> T;
    };
}
constexpr auto transform2(const Callable<size_t>& callable) {
    constexpr auto v = callable();
    return std::integral_constant<decltype(v), v>{};
}

template<typename T>
constexpr auto transform3(T v) {
    return std::integral_constant<typename T::value_type, v[0] + v[v.size - 1]>{};
}

constexpr auto transform4(const auto& callable) {
    constexpr auto v = callable();
    typedef decltype(v) T;
    return std::integral_constant<typename T::value_type, v[0] + v[v.size - 1]>{};
}


constexpr auto calculateSomeMagic(auto v) {
    std::array<uint8_t, 10> array;
    for(auto& e : array) {
        e = v;
    }
    return array;
}

namespace  {
    constexpr auto a = calculateSomeMagic(42);
    
//    auto b = transform3(a); // NOK
    auto b = transform4([&]{return a;});
}


int main() {
    constexpr int value = 42;

//        constexpr auto result1 = transform1(value);  // not possible
    
    [[maybe_unused]] constexpr auto result2 = transform2([&]{return value;}); // constant lambda argument wrapper

//    transform2(value); // constraint not satisfied
//    transform2([&]{return std::byte{0};}); // constraint not satisfied
}