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

#include <cstdint>
#include <cstddef>

#include <std/chrono>
#include <etl/rational.h>
#include <etl/concepts.h>
#include <etl/algorithm.h>

#include "mcu/common/concepts.h"

#include "tick.h"

namespace External {
    template<typename Pin1, typename Pin2, typename ValueType = uint8_t, ValueType d = ValueType{}>
    class RotaryEncoder {
    public:
        typedef ValueType value_type;
        typedef Pin1 pin1_type;
        typedef Pin2 pin2_type;
        static void init() {
            Pin1::template dir<AVR::Input>();
            Pin2::template dir<AVR::Input>();
            Pin1::template pullup<true>();
            Pin2::template pullup<true>();
        }
        static void start() {}

        static inline void rateProcess() {
            uint8_t newState = (Pin1::read() ? 2 : 0) + (Pin2::read() ? 1 : 0);    
            switch (mLastState) {
            case 0:
                if (newState == 1) {
                    ++mValue;
                }
                else if (newState == 2) {
                    --mValue;
                }
                break;
            case 1:
                if (newState == 0) {
                    --mValue;
                }
                break;
            case 2:
                if (newState == 0) {
                    ++mValue;
                }
                break;
            case 3:
                break;
            default:
                assert(false);
            }
            mLastState = newState;
        }
        static ValueType value() {
            return mValue;
        }   
    private:
        inline static ValueType mValue{d};
        inline static uint8_t mLastState = 0;
    };
}
