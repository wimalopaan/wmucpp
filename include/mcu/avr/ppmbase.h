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

template<typename Timer, typename Prescaler = void>
class PPMParameter {
    PPMParameter() = delete;
public:
    static constexpr uint32_t prescaler = [](){
        if constexpr(std::is_same<void, Prescaler>::value) {
            return AVR::Util::calculatePpmOutParameter<Timer, uint16_t>();
        }
        else {
            return Prescaler::value;
        }
    }();
    static_assert(prescaler > 0, "wrong prescaler");
    static constexpr std::hertz timerFrequency = Config::fMcu / prescaler;
    static constexpr uint16_t ocMin = 1_ms * timerFrequency;
    static_assert(ocMin > 0, "wrong oc value");
    static constexpr uint16_t ocMax = 2_ms * timerFrequency;
    static_assert(ocMax > 0, "wrong oc value");
    static constexpr uint16_t ocDelta = ocMax - ocMin;
    static_assert(ocDelta > 0, "wrong oc value");
    static constexpr uint16_t ocFrame = 20_ms * timerFrequency;
    static_assert(ocFrame > 0, "wrong oc value");

    static constexpr uint16_t ccPulse = 500_us * timerFrequency;
    static_assert(ccPulse > 0, "wrong oc value");
};

//template<typename Timer>
//constexpr std::hertz PPMParameter<Timer>::timerFrequency;
