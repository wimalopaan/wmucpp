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

#include "hal/event.h"

template<int N, typename Decoder>
class PpmSwitch final {
    enum class State {Undefined, Low, Medium, High};

public:
    PpmSwitch() = delete;
    inline constexpr static uint8_t number = N;
    typedef Decoder decoder_type;

    static void init() {
        state = State::Undefined;
    }
    static void process(uint8_t ppmValue) {
        switch(state) {
        case State::Undefined:
            if ((ppmValue >= Decoder::ppmMidLow) && (ppmValue <= Decoder::ppmMidHigh)) {
                state = State::Medium;
            }
            else if (ppmValue <= Decoder::ppmMinHigh) {
                state = State::Low;
            }
            else if (ppmValue >= Decoder::ppmMaxLow) {
                state = State::High;
            }
            break;
        case State::Low:
            if (ppmValue >= Decoder::ppmMidLow) {
                state = State::Medium;
                EventManager::enqueue({EventType::Ppm1Down, std::byte{ppmValue}});
            }
            break;
        case State::Medium:
            if (ppmValue <= Decoder::ppmMinHigh) {
                state = State::Low;
            }
            if (ppmValue >= Decoder::ppmMaxLow) {
                state = State::High;
            }
            break;
        case State::High:
            if (ppmValue <= Decoder::ppmMidHigh) {
                state = State::Medium;
                EventManager::enqueue({EventType::Ppm1Up, std::byte{ppmValue}});
            }
            break;
        default:
            break;
        }
    }

private:
    // fixme: in GPIOR -> Jeder State ein Bit im FlagRegister
    inline static State state = State::Undefined;
};
