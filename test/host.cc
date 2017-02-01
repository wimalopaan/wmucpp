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

#include "util/bits.h"

#include <numeric>
#include <array>
#include <utility>
#include <cassert>

constexpr int foo(size_t v) {
    return v + 0x30;
}

template<typename F, size_t... II>
constexpr auto createArray(F f, std::index_sequence<II...>) {
    return std::array<int, sizeof...(II)> {{f(II)...,}};
}
 

//int main(){
//  std::array<char, 120> txText{"radio tx \0"};
//  std::array<char, 20>  dBuffer{"35929910576AB235\0"};

//  std::copy(std::begin(dBuffer), std::end(dBuffer), std::begin(txText) + 9);

//  printf("\n%s", &txText[0]);

//  return 0;

//}


//int main()
//{
//    constexpr auto a = createArray(foo, std::make_index_sequence<10>{});
////    constexpr auto b = createArray([](size_t){return 1;}, std::make_index_sequence<10>{});
    
//    std::copy(std::begin(a), std::end(a), std::ostream_iterator<double>(std::cout, ", "));
    
////    constexpr auto a = createArray2(foo, 1, 2);
    
    
////    constexpr FixedPoint<int16_t, 4> f1 = 1.75_fp;          
////    constexpr FixedPoint<int16_t, 4> f2(0.5);          
////    constexpr FixedPoint<int16_t, 4> f3(-0.5);
    
////    std::cout << f3.raw() << std::endl;
////    std::cout << f3.integer() << std::endl;
////    std::cout << f3.fractional() << std::endl;

////    constexpr FixedPoint<int16_t, 4> f4(-1.5);
////    std::cout << f4.integer() << std::endl;
////    std::cout << f4.fractional() << std::endl;
    
//}

void assertFunction(bool b, const char *function, const char *file, unsigned int line) {
    assert(b);
    if (!b) {
        std::cout << "Assertion failed in: " << function << " file: " << file << " line: " << line << std::endl;
    }
}

//enum class A {
//    a = 1 << 0, 
//    b = 1 << 1, 
//    c = 1 << 2
//};

//template<typename T, T... Values>
//struct static_container final {};

//template<typename T, typename... Ts>
//constexpr auto make_static_container(T t0, Ts... ts) {
//    return static_container<T, t0, ts...>{};
//}

//template<typename T, T... Values>
//constexpr auto make_static_container(std::integral_constant<T, Values>...) noexcept {
//    return static_container<T, Values...>{};
//}

//template<typename T>
//std::integral_constant(T v) -> std::integral_constant<T, v>;

//template<A... Flags>
//void inline set() {
//    std::cout << sizeof... (Flags) << std::endl;
//}

//template<typename F, F... FF>
//void inline set(static_container<F, FF...>) {
//    std::cout << sizeof... (FF) << std::endl;
//}


//int main() {
//    constexpr static_container<A, A::a, A::b> sc1{};
////    constexpr auto sc2 = make_static_container(A::a, A::c);
 
//    set(sc1);    
////    set(sc2);    
//}

//enum class AlarmFlags : uint8_t {
//    NoTimer =    0,
//    Periodic =   1 << 0,
//    OneShot  =   1 << 1,
//    AutoDelete = 1 << 2,
//    Disabled   = 1 << 3
//};

//namespace std {
//template<>
//struct enable_bitmask_operators<AlarmFlags>{
//    static const bool enable = true;
//};
//}

//int main() {
//    AlarmFlags f1 = AlarmFlags::NoTimer;
//    AlarmFlags f2 = AlarmFlags::Periodic;

//    AlarmFlags f3 = AlarmFlags::Disabled | AlarmFlags::AutoDelete;
    
//    std::cout << static_cast<int>(f2 | AlarmFlags::Disabled) << std::endl;    
//    std::cout << static_cast<int>(f3) << std::endl;    
//}

struct AT1 {
    static constexpr int x = 0;
};

struct AT2 {
    static constexpr int y = 0;
};

struct AT3 {
    static constexpr int x = 0;
};

template<typename T>
struct Part {
    static constexpr uint8_t x = T::x;
};

template<typename T>
struct Feature {
    static constexpr Part<T> b;
};

template<>
struct Feature<AT2> {
    static constexpr int b = 0;
};

volatile int q = 0;

int main()
{
    Feature<AT1> x1;
    q = x1.b.x;
    Feature<AT2> x2;
    q = x2.b;
    Feature<AT3> x3;
    q = x3.b.x;
}
