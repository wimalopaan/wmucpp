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

#pragma once

#include "config.h"
#include "mcu/ports.h"
#include "hal/event.h"

template<typename Pin1, typename Pin2, typename ValueType = uint8_t>
class RotaryEncoder {
public:
    typedef ValueType value_type;
    typedef Pin1 pin1_type;
    typedef Pin2 pin2_type;
    static void init() {
        Pin1::template dir<AVR::Input>();
        Pin2::template dir<AVR::Input>();
        Pin1::pullup();
        Pin2::pullup();
    }
    static void start() {}
    static inline void rateProcess() {
        uint8_t newState = (Pin1::read() ? 2 : 0) + (Pin2::read() ? 1 : 0);    
        switch (mLastState) {
        case 0:
            if (newState == 1) {
                mValue += 1;
            }
            else if (newState == 2) {
                mValue -= 1;
            }
            break;
        case 1:
            if (newState == 0) {
                mValue -= 1;
            }
            break;
        case 2:
            if (newState == 0) {
                mValue += 1;
            }
            break;
        case 3:
            break;
        default:
            assert(false);
        }
//        uint8_t offset = transitionTable[mLastState][newState];
//        mValue += offset;
        mLastState = newState;
    }
    static ValueType value() {
        return mValue;
    }   
private:
    inline static ValueType mValue = 0;
    inline static uint8_t mLastState = 0;
//    static inline constexpr uint8_t numberOfStates = 4;
//    static inline constexpr int8_t transitionTable[numberOfStates][numberOfStates] = {
//        { 0, 1, -1, 0},
//        {-1, 0,  0, 0},
//        { 1, 0,  0, 0},
//        { 0, 0,  0, 0}
//    };
};
