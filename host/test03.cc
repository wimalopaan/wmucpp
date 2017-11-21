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
#include <vector>
#include <array>
#include <memory>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"

//struct A {
//    static constexpr bool hasBar = false;
//};

template<typename T>
void foo() {
    if constexpr (T::hasBar) {
        T::bar();
    }    
}

size_t nextSNR() {
    static size_t snr = 0;
    return ++snr;
}

struct A {
    A(int v = 0) : mValue{v}, mSNR(nextSNR()) {} // Code Duplizierung, reorder Problem
    A(double v) : A{static_cast<int>(v)} {}  // Code Duplizierung
    A(const A& o) : A{o.mValue} {} // mValue vergessen, Code Duplizierung
    
    int mValue; 
    const size_t mSNR;
    static const std::pair<int, float> mMagic;
};
const std::pair<int, float> A::mMagic{1, 1.0f};

struct B {
    B() : mSNR{nextSNR()} {} // mValue vergessen (Absicht?), Code Duplizierung
    B(int v) : mSNR{nextSNR()}, mValue{v} {} // Code Duplizierung, reorder Problem
    B(double v) : mValue{static_cast<int>(v)}, mSNR{nextSNR()} {}  // Code Duplizierung
    B(const B&) : mSNR{nextSNR()} {} // mValue vergessen, Code Duplizierung
    
    int mValue; 
    const size_t mSNR;
    static const std::pair<int, float> mMagic;
    int* mPtr; // Assertionen ?
};
const std::pair<int, float> B::mMagic{1, 1.0f};

struct C {
    C() = default;
    C(int v) : mValue{v} {}
    C(double v) : mValue{static_cast<int>(v)} {}
    C(const C& o) : mValue{o.mValue} {}
    
    int mValue = 0; // Idiom: Variablen werden dort initialisiert, wo sie definiert werden
    const size_t mSNR = nextSNR(); // s.o.
    inline static const auto mMagic = std::pair(1, 1.0f); // s.o., auto-deduction-guide möglich
    int* mPtr = nullptr; // <> Assertionen schlagen fehl
};

struct D {
    D(int v = 0) : mValue{v} {}
    D(double v) : D{static_cast<int>(v)} {}
    D(const D& o) : D{o.mValue} {}
    
    int mValue = 0; // Idiom: Variablen werden dort initialisiert, wo sie definiert werden
    const size_t mSNR = nextSNR(); // s.o.
    inline static const auto mMagic = std::pair(1, 1.0f); // s.o., auto-deduction-guide möglich
};

typedef uint32_t Event_t;

uint32_t foo(uint16_t x) {
    return 2 * x;
}

struct EventSafe {
    uint32_t value;
};

std::unique_ptr<A> fox() {
  auto ret = std::make_unique<A>();
  
  ret->mValue = 0;
  
  return std::move(ret);
}

int main() {
//    Event_t e1 = 1;
//    foo(e1);
    
//    EventSafe e2 = {2};
//    foo(e2);
    
    auto a = fox();
    
    B b1;
    B b2 = b1;
    C c1(42);
    C c2 = c1;
    
    std::cout << b1.mSNR << ' ' << b1.mValue << ' ' << b1.mMagic.first << std::endl;
    std::cout << b2.mSNR << ' ' << b2.mValue << ' ' << b2.mMagic.first << std::endl;
    std::cout << c1.mSNR << ' ' << c1.mValue << ' ' << c1.mMagic.first << std::endl;
    std::cout << c2.mSNR << ' ' << c2.mValue << ' ' << c2.mMagic.first << std::endl;
}

#pragma GCC diagnostic pop