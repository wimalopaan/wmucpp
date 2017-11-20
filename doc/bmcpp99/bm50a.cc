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

#define NDEBUG

#include <stdlib.h>
#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/mcutimer.h"
#include "std/array"
#include "std/utility"
#include "std/algorithm"
#include "util/disable.h"
#include "util/concepts.h"
#include "util/sort.h"
#include "container/stringbuffer.h"
#include "simavr/simavrdebugconsole.h"
#include "console.h"

using terminalDev = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDev>;

using std::array;

using constantRateTimer = AVR::Timer16Bit<1>;
constexpr const auto constantRatePeriod = 1000_us;

struct CTHandler : public IsrBaseHandler<AVR::ISR::Timer<1>::CompareA> {
    static void isr() {
        ++mCounter;
    }
    inline static volatile uint16_t mCounter = 0;
};

using isrRegistrar = IsrRegistrar<CTHandler>;

struct Measure {
    std::milliseconds time;
    StringBuffer<20> name;
    
};
const uint32_t iterations = 100;
std::array<Measure, 20> times;

constexpr uint8_t N = 191;
array<uint8_t, N> v1;
array<uint16_t, N> v2;
array<uint32_t, N> v3;

//array<StringBuffer<10>, 100> v3;
//// proxy array wird sortiert
//template<uint8_t N>
//bool operator<(const StringBuffer<N>& a, const StringBuffer<N>& ) {

//}

template<typename T>
int less(const void* a, const void* b) {
    const T* a1 = static_cast<const T*>(a);
    const T* b1 = static_cast<const T*>(b);
    return *a1 - *b1;
}

volatile uint8_t y = 0;

uint8_t filler(size_t i) {
//    return (N - i) * 31; // 1
    return (N - i) % 31; // 2
}

//typedef sort::Algorithm::QuickNaiv Algo;
//typedef sort::Algorithm::QuickSedgeWick Algo;
typedef sort::Algorithm::QuickBentleyMcIlroy Algo;

int main() {
    isrRegistrar::init();
    Scoped<EnableInterrupt<>> interruptEnabler;
    
    constexpr std::hertz constantRateFrequency = 1 / constantRatePeriod;
    constexpr auto tsd = AVR::Util::calculate<constantRateTimer>(constantRateFrequency);
    static_assert(tsd, "wrong parameter");
    constantRateTimer::prescale<tsd.prescaler>();
    constantRateTimer::ocra<tsd.ocr>();
    constantRateTimer::mode(AVR::TimerMode::CTC);
    
    uint8_t algo = 0;
    
    {
        uint32_t start = CTHandler::mCounter;
        for(uint32_t n = 0; n < iterations; ++n) {
            for (size_t i = 0; i < v1.size; i++) {
                v1[i] = filler(i);
            }
        }
        uint32_t end = CTHandler::mCounter;           
        times[algo].name.insertAt(0, "empty"_pgm);
        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
    }    
//    {
//        uint32_t start = CTHandler::mCounter;
//        for(uint32_t n = 0; n < iterations; ++n) {
//            for (size_t i = 0; i < v1.size; i++) {
//                v1[i] = filler(i);
//            }
//            sort::quickSort<Algo>(v1);
//        }
//        uint32_t end = CTHandler::mCounter;           
//        times[algo].name.insertAt(0, "q8"_pgm);
//        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
//    }    
//    {
//        uint32_t start = CTHandler::mCounter;
//        for(uint32_t n = 0; n < iterations; ++n) {
//            for (size_t i = 0; i < v2.size; i++) {
//                v2[i] = filler(i);
//            }
//            sort::quickSort<Algo>(v2);
//        }
//        uint32_t end = CTHandler::mCounter;           
//        times[algo].name.insertAt(0, "q16"_pgm);
//        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
//    }    
//    {
//        uint32_t start = CTHandler::mCounter;
//        for(uint32_t n = 0; n < iterations; ++n) {
//            for (size_t i = 0; i < v3.size; i++) {
//                v3[i] = filler(i);
//            }
//            sort::quickSort<Algo>(v3);
//        }
//        uint32_t end = CTHandler::mCounter;           
//        times[algo].name.insertAt(0, "q32"_pgm);
//        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
//    }    
    {
        uint32_t start = CTHandler::mCounter;
        for(uint32_t n = 0; n < iterations; ++n) {
            for (size_t i = 0; i < v1.size; i++) {
                v1[i] = filler(i);
            }
            qsort(&v1[0], v1.size, sizeof(v1[0]), less<uint8_t>);
        }
        uint32_t end = CTHandler::mCounter;           
        times[algo].name.insertAt(0, "l8"_pgm);
        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
    }    
    {
        uint32_t start = CTHandler::mCounter;
        for(uint32_t n = 0; n < iterations; ++n) {
            for (size_t i = 0; i < v2.size; i++) {
                v2[i] = filler(i);
            }
            qsort(&v2[0], v2.size, sizeof(v2[0]), less<uint16_t>);
        }
        uint32_t end = CTHandler::mCounter;           
        times[algo].name.insertAt(0, "l16"_pgm);
        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
    }    
    {
        uint32_t start = CTHandler::mCounter;
        for(uint32_t n = 0; n < iterations; ++n) {
            for (size_t i = 0; i < v3.size; i++) {
                v3[i] =  filler(i);
            }
            qsort(&v3[0], v3.size, sizeof(v3[0]), less<uint32_t>);
        }
        uint32_t end = CTHandler::mCounter;           
        times[algo].name.insertAt(0, "l32"_pgm);
        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
    }    
    {
        uint32_t start = CTHandler::mCounter;
        for(uint32_t n = 0; n < iterations; ++n) {
            for (size_t i = 0; i < v1.size; i++) {
                v1[i] = filler(i);
            }
            sort::bubbleSort(v1);
        }
        uint32_t end = CTHandler::mCounter;           
        times[algo].name.insertAt(0, "b8"_pgm);
        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
    }    
    if constexpr(std::is_same<Algo, sort::Algorithm::QuickNaiv>::value) {
        std::outl<terminal>("QuickNaiv"_pgm);
    }
    if constexpr(std::is_same<Algo, sort::Algorithm::QuickSedgeWick>::value) {
        std::outl<terminal>("QuickSedgewick"_pgm);
    }
    if constexpr(std::is_same<Algo, sort::Algorithm::QuickBentleyMcIlroy>::value) {
        std::outl<terminal>("QuickBMcI"_pgm);
    }

    for(uint8_t i = 0; i < algo; ++i) {
        std::outl<terminal>(times[i].name, " : "_pgm, (times[i].time - times[0].time));
    }        
    
//        for(auto x : v1) {
//            std::outl<terminal>(x);
//        }
}

ISR(TIMER1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("ass"_pgm, line);
    while(true) {}
}
#endif
