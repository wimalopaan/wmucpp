/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "mcu/avr/adc.h"
#include "util/fixedpoint.h"

template<typename Controller, uint8_t Index>
class LM35 {
public:
    typedef Controller controller_type;
    static constexpr uint8_t index = Index;
    
    static constexpr double VoltPerDegree{0.01};
    static constexpr auto VBit = Controller::mcu_adc_type::VBit;
    static constexpr FixedPoint<uint16_t, 8> degreeScale{VBit / VoltPerDegree};
    
    static FixedPoint<uint16_t, 8> temperature() {
        return FixedPoint<uint16_t, 8>::fromRaw(degreeScale.raw() * Controller::value(index));
    }
};
