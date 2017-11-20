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

#pragma once

#include <stdint.h>
#include "units/percent.h"
#include "util/dassert.h"

namespace HAL {
    template<typename PinSet, typename ValueType = uint16_t, ValueType Top = 0>
    class SoftPWM {
        template<typename PinList> 
        struct Checker;
        template<typename... NumberedPins>
        struct Checker<Meta::List<NumberedPins...>> {
            static void check() {
                ((freeCounter >= mThresh[NumberedPins::value] ? NumberedPins::type::off() : (void)0), ...);
            }
        };
        template<typename T, size_t Number> struct Numberer{
            typedef T type;
            static inline constexpr size_t value = Number;
        };
        using numberedPinList = Meta::transformN<Numberer, typename PinSet::pinlist>;
        using checker = Checker<numberedPinList>;
    public:
        typedef PinSet pin_set;
        typedef ValueType value_type;
        
        static void init() {
            pin_set::template dir<AVR::Output>();
            pin_set::allOff();    
        }
        static void freeRun() {
            if constexpr(Top == 0) {
                ++freeCounter;
            }
            else {
                freeCounter = (freeCounter + 1) % Top;
            }
            if (freeCounter == 0) {
                PinSet::allOn();
            }
            else {
                checker::check();
            }
        }
        static void pwm(const std::percent& p, uint8_t index) {
            assert(index < PinSet::size);
            mThresh[index] = std::expand(p, value_type{0}, value_type{Top - 1});
        }
    private:    
        inline static value_type mThresh[PinSet::size] = {};
        inline static value_type freeCounter = 0;
    };
}


//template<typename... Pins>
//class SoftPWM {
//    // todo: besser mit constexpr if
//    template<uint8_t N, typename P, typename... PP>
//    struct Checker {
//        static void checkOff(uint16_t v) {
//            if (v > SoftPWM<Pins...>::mThresh[N]) {
//                P::low();
//            }
//            Checker<N + 1, PP..., void>::checkOff(v);
//        }  
//        static void checkOn() {
//            if (SoftPWM<Pins...>::mThresh[N] > 0) {
//                P::high();
//            }
//            Checker<N + 1, PP..., void>::checkOn();
//        }  
//    };
//    template<typename... PP>
//    struct Checker<sizeof...(Pins), void, PP...> {
//        static void checkOff(uint16_t) {}  
//        static void checkOn() {}  
//    };
    
//public:
//    static void init() {
//        (Pins::template dir<AVR::Output>(),...);
//        (Pins::low(),...);
//    }
//    static void start() {}
//    static void freeRun() {
//        ++freeCounter;
//        Checker<0, Pins...>::checkOff(freeCounter);
//    }
//    static void periodic() {
//        mPeriod = std::max(freeCounter, mPeriod);
//        freeCounter = 0;
//        Checker<0, Pins...>::checkOn();
//    }
//    static constexpr auto rateProcess = periodic;
    
//    static const volatile uint16_t& period() {
//        return mPeriod;
//    }
//    static void pwm(const std::percent& p, uint8_t index) {
//        assert(index < sizeof...(Pins));
//        mThresh[index] = std::expand(p, 0U, mPeriod);
//    }
    
//private:
//    inline static uint16_t mThresh[sizeof...(Pins)] = {};
//    inline static uint16_t mPeriod = 0;
//    inline static uint16_t freeCounter = 0;
//};
