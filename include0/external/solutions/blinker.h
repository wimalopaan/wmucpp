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
#include <etl/types.h>

#include "mcu/common/concepts.h"

namespace External {

    template<typename Pin, const std::chrono::milliseconds& Intervall, const std::chrono::milliseconds& Pulse, const std::chrono::milliseconds& Period>
    struct Blinker {
        enum class State : uint8_t {Undefined, Off, Steady, Blink, _Number};
        
        static_assert(Pulse <= Period, "wrong parameter");
        
        static inline constexpr auto pulseWidthCount = Pulse / Intervall;
        static inline constexpr auto periodWidthCount = Pulse / Intervall;
        
        using value_type = etl::typeForValue_t<periodWidthCount>;
//        value_type::_;
        
        static inline constexpr auto maxDisplayablePulses = periodWidthCount / (2 * pulseWidthCount);
        
        static inline void init() {
            Pin::init();
        }
        static inline void off() {
            mState = State::Off;
        }
        static inline void steady() {
            mState = State::Steady;
        }
        static inline void blink(etl::uint_ranged<uint8_t, 0, maxDisplayablePulses> count) {
            mState = State::Blink;
            mBlinkCount = count;
        }
        static inline void periodic() {
            switch(mState) {
            case State::Undefined:
                Pin::inactivate();
                mState = State::Off;
                break;
            case State::Off:
                Pin::inactivate();
                break;
            case State::Steady:
                Pin::activate();
                break;
            case State::Blink:
                if (shouldBlink()) {
                    auto phase = mStateCounter.toInt() % (2 * pulseWidthCount);
                    if (phase < pulseWidthCount) {
                        Pin::activate();
                    }
                    else {
                        Pin::inactivate();
                    }
                }
                else {
                    Pin::inactivate();
                }
                break;
            default:
                break;
            }
        }
    private:
        static inline bool shouldBlink() {
            return mStateCounter.toInt() < (2 * pulseWidthCount * mBlinkCount.toInt());
        }
        static inline State mState = State::Off;
        static inline etl::uint_ranged_circular<value_type, 0, periodWidthCount + 1> mStateCounter;
        static inline etl::uint_ranged<uint8_t, 0, maxDisplayablePulses> mBlinkCount;
    };
}
