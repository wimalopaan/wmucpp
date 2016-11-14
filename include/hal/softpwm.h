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

#pragma once

#include <stdint.h>
#include "units/percent.h"
#include "util/dassert.h"

template<typename... Pins>
class SoftPWM {
    template<uint8_t N, typename P, typename... PP>
    struct Checker {
        static void checkOff(uint16_t v) {
            if (v > SoftPWM<Pins...>::mThresh[N]) {
                P::low();
            }
            Checker<N + 1, PP..., void>::checkOff(v);
        }  
        static void checkOn() {
            if (SoftPWM<Pins...>::mThresh[N] > 0) {
                P::high();
            }
            Checker<N + 1, PP..., void>::checkOn();
        }  
    };
    template<typename... PP>
    struct Checker<sizeof...(Pins), void, PP...> {
        static void checkOff(uint16_t) {}  
        static void checkOn() {}  
    };
    
public:
    static void init() {
        (Pins::template dir<AVR::Output>(),...);
        (Pins::low(),...);
    }
    static void freeRun() {
        ++freeCounter;
        Checker<0, Pins...>::checkOff(freeCounter);
    }
    static void periodic() {
        mPeriod = std::max(freeCounter, mPeriod);
        freeCounter = 0;
        Checker<0, Pins...>::checkOn();
    }
    static const volatile uint16_t& period() {
        return mPeriod;
    }
    static void pwm(const std::percent& p, uint8_t index) {
        assert(index < sizeof...(Pins));
        mThresh[index] = std::expand(p, 0U, mPeriod);
    }
    
private:
    static uint16_t mThresh[sizeof...(Pins)];
    static uint16_t mPeriod;
    static uint16_t freeCounter;
};

template<typename... Pins>
uint16_t SoftPWM<Pins...>::freeCounter = 0;
template<typename... Pins>
uint16_t SoftPWM<Pins...>::mPeriod = 0;
template<typename... Pins>
uint16_t SoftPWM<Pins...>::mThresh[sizeof...(Pins)] = {};
