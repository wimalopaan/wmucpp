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

// todo: abstrahieren als Algorithmus
// First
// ForEach
// s.a. SoftPPM

template<typename... Pins>
class SoftPWM {
public:
    static void init() {
        (Pins::template dir<AVR::Output>(),...);
        (Pins::low(),...);
    }

    static void freeRun() {
        ++freeCounter;
        if (freeCounter > (mPeriod / 2)) {
            (Pins::low(),...);
        }
    }

    static void periodic() {
        mPeriod = freeCounter;
        freeCounter = 0;
        (Pins::high(),...);
    }

    static const volatile uint16_t& period() {
        return mPeriod;
    }

private:
    static volatile uint16_t mPeriod;
    static volatile uint16_t freeCounter;
};

template<typename... Pins>
volatile uint16_t SoftPWM<Pins...>::freeCounter = 0;
template<typename... Pins>
volatile uint16_t SoftPWM<Pins...>::mPeriod = 0;
