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
    template<AVR::Concepts::Activatable Pin, typename Timer, auto ShortPressTicks, auto LongPressTicks>
    struct ButtonRelese {
        enum class Press : uint8_t {Short, Long, None};
        enum class State : uint8_t {PressedShort, PressedLong, Released, Wait};
        
        static_assert(ShortPressTicks < LongPressTicks);
        
        static inline void init() {
            Pin::init();
        }
        static inline void ratePeriodic() {
            const auto oldState = mState;
            const auto on = Pin::isActive();
            ++mStateCounter;
            switch(mState) {
            case State::Released:
                if (on) {
                    mState = State::Wait;
                }
                break;
            case State::Wait:
                if (!on) {
                    mState = State::Released;
                }
                mStateCounter.on(ShortPressTicks, []{
                    mState = State::PressedShort;
                });
                break;
            case State::PressedShort:
                if (!on) {
                    mState = State::Released;
                }
                mStateCounter.on(LongPressTicks, []{
                    mState = State::PressedLong;
                });
                break;
            case State::PressedLong:
                if (!on) {
                    mState = State::Released;
                }
                break;
            }
            if (oldState != mState) {
                mStateCounter.reset();
                switch(mState) {
                case State::Released:
                    if (oldState == State::PressedShort) {
                        mEvent = Press::Short;
                    }
                    else if (oldState == State::PressedLong) {
                        mEvent = Press::Long;
                    }
                    break;
                case State::PressedShort:
                    break;
                case State::PressedLong:
                    break;
                case State::Wait:
                    break;
                }
            }
        }
        static inline Press event() {
            auto e = Press::None;
            std::swap(e, mEvent);
            return e;
        }
        static inline bool pressed() {
            return (mState == State::PressedLong) || (mState == State::PressedShort); 
        }
private:        
        static inline Press mEvent{Press::None};
        static inline State mState{State::Released};
        static inline Tick<Timer> mStateCounter;
    };


    template<AVR::Concepts::Activatable Pin, typename Timer, auto ShortPressTicks, auto LongPressTicks>
    struct Button2 {
        enum class Press : uint8_t {Short, Long, Release, None};
        enum class State : uint8_t {PressedShort, PressedLong, Released, Wait};
        
        static_assert(ShortPressTicks < LongPressTicks);
        
        static inline void init() {
            Pin::init();
        }
        static inline void ratePeriodic() {
            const auto oldState = mState;
            const auto on = Pin::isActive();
            ++mStateCounter;
            switch(mState) {
            case State::Released:
                if (on) {
                    mState = State::Wait;
                }
                break;
            case State::Wait:
                if (!on) {
                    mState = State::Released;
                }
                mStateCounter.on(ShortPressTicks, []{
                    mState = State::PressedShort;
                });
                break;
            case State::PressedShort:
                if (!on) {
                    mState = State::Released;
                }
                mStateCounter.on(LongPressTicks, []{
                    mState = State::PressedLong;
                });
                break;
            case State::PressedLong:
                if (!on) {
                    mState = State::Released;
                }
                break;
            }
            if (oldState != mState) {
                mStateCounter.reset();
                switch(mState) {
                case State::Released:
                    if (oldState != State::Wait) {
                        mEvent = Press::Release;
                    }
                    break;
                case State::PressedShort:
                    mEvent = Press::Short;
                    break;
                case State::PressedLong:
                    mEvent = Press::Long;
                    break;
                case State::Wait:
                    break;
                }
            }
        }
        static inline Press event() {
            auto e = Press::None;
            std::swap(e, mEvent);
            return e;
        }
        static inline bool pressed() {
            return (mState == State::PressedLong) || (mState == State::PressedShort); 
        }
private:        
        static inline Press mEvent{Press::None};
        static inline State mState{State::Released};
        static inline Tick<Timer> mStateCounter;
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
            periodic([&](const auto v){
                if (v != Press::None) {
                    mEvent = v;
                }
            });
        }
        static inline void ratePeriodic() {
            periodic();
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
