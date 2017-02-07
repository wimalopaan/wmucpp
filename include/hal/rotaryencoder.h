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

#include "config.h"
#include "mcu/ports.h"
#include "hal/event.h"

template<typename Pin1, typename Pin2, typename ValueType = uint8_t>
class RotaryEncoder {
public:
    typedef ValueType value_type;
    static void init() {
        Pin1::template dir<AVR::Input>();
        Pin2::template dir<AVR::Input>();
        Pin1::pullup();
        Pin2::pullup();
    }
    static void rateProcess() {
        uint8_t newState = (Pin1::read() ? 2 : 0) + (Pin2::read() ? 1 : 0);    
        uint8_t offset = transitionTable[lastState()][newState];
        value() += offset;
        lastState() = newState;
    }
    static ValueType& value() {
        static ValueType v = 0;        
        return v;
    }   

private:
    static uint8_t& lastState() {
        static uint8_t ls = 0;
        return ls;
    }
    static constexpr uint8_t numberOfStates = 4;
    static constexpr int8_t transitionTable[numberOfStates][numberOfStates] = {
        { 0, 1, -1, 0},
        {-1, 0,  0, 0},
        { 1, 0,  0, 0},
        { 0, 0,  0, 0}
    };
};
