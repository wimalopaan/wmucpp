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
    template<AVR::Concepts::Activatable Pin, typename Timer, auto ShortPressTicks, auto LongPressTicks, auto UltraLongPressTicks>
    struct ExtendedButton {
        
    };

    template<AVR::Concepts::Activatable Pin, typename Timer, auto ShortPressTicks, auto LongPressTicks>
    struct Button {
        enum class Press : uint8_t {Short, Long, Release, None};
        enum class State : uint8_t {Pressed, Released};
        
        static_assert(ShortPressTicks < LongPressTicks);
        
        static inline void init() {
            Pin::init();
        }
        template<etl::Concepts::Callable<Press> F>
        static inline void periodic(const F& f) {
            if (Pin::isActive()) {
                if (mCounter > ShortPressTicks) {
                    mState = State::Pressed;
                    if (mCounter > LongPressTicks) {
                        f(Press::Long);
                        return;
                    }
                    else {
                        f(Press::Short);
                    }
                }
                ++mCounter;
            }
            else {
                mCounter.reset();
                if (mState == State::Pressed) {
                    f(Press::Release);
                }
                mState = State::Released;
            }
        }
        static inline void periodic() {
            periodic([&](auto v){
                if (v != Press::None) {
                    mEvent = v;
                }
            });
        }
        static inline Press event() {
            auto e = Press::None;
            std::swap(e, mEvent);
            return e;
        }
        static inline State state() {
            return mState;
        }
        static inline void reset() {
            mCounter.reset();
        }
    private:
        static inline Press mEvent{Press::None};
        static inline State mState{State::Released};
        static inline Tick<Timer> mCounter;
    };
}
