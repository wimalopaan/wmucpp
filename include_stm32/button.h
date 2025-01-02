/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 -2024 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "tick.h"

namespace External {
    template<typename Pin, typename Timer, auto ShortPressTicks, auto LongPressTicks, typename Debug = void>
    struct Button {
        enum class Press : uint8_t {Short, Long, Release, None};
        enum class State : uint8_t {PressedShort, PressedLong, Released, Wait};

        static_assert(ShortPressTicks < LongPressTicks);

        static inline constexpr auto shortPressTicks = External::Tick<Timer>::fromRaw(ShortPressTicks);
        static inline constexpr auto longPressTicks = External::Tick<Timer>::fromRaw(LongPressTicks);

        static inline void init() {
            Pin::template pullup<true>();
            Pin::template dir<Mcu::Input>();
        }
        static inline void ratePeriodic() {
            const auto oldState = mState;
            const bool on = !Pin::read();
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
                mStateCounter.on(shortPressTicks, []{
                    mState = State::PressedShort;
                });
                break;
            case State::PressedShort:
                if (!on) {
                    mState = State::Released;
                }
                mStateCounter.on(longPressTicks, []{
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
                case State::Wait:
                    IO::outl<Debug>("Wait");
                    break;
                case State::Released:
                    IO::outl<Debug>("Released");
                    if (oldState != State::Wait) {
                        mEvent = Press::Release;
                    }
                    break;
                case State::PressedShort:
                    IO::outl<Debug>("Short");
                    mEvent = Press::Short;
                    break;
                case State::PressedLong:
                    IO::outl<Debug>("Long");
                    mEvent = Press::Long;
                    break;
                }
            }
        }
        static inline Press event() {
            return std::exchange(mEvent, Press::None);
        }
        static inline bool pressed() {
            return (mState == State::PressedLong) || (mState == State::PressedShort);
        }
private:
        static inline State mState{State::Released};
        static inline Tick<Timer> mStateCounter;
        static inline Press mEvent{Press::None};
    };
}
