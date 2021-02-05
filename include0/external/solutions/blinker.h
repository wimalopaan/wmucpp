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

#include <external/solutions/tick.h>
#include <std/chrono>
#include <etl/rational.h>
#include <etl/concepts.h>
#include <etl/algorithm.h>
#include <etl/types.h>

#include "mcu/common/concepts.h"

namespace External {
    template<typename Pin, typename Timer, auto PulseWidth>
    struct SimpleBlinker {
        enum class State : uint8_t {Inactive, Off, On};
        static constexpr External::Tick<Timer> pulseTicks{PulseWidth};
    
        static inline void init() {
            Pin::init();
//            Pin::template dir<AVR::Output>();
        }
        static inline void steady() {
            mState = State::Inactive;
            Pin::activate();
        }
        static inline void off() {
            mState = State::Inactive;
            Pin::inactivate();
        }
        static inline void ratePeriodic() {
            const auto oldstate = mState;
            ++stateTicks;
            switch(mState) {
            case State::Inactive:   
                break;
            case State::On:   
                Pin::activate();
                stateTicks.on(pulseTicks, []{
                    ++mNumber;
                    mState = State::Off;
                });
                break;
            case State::Off:   
                Pin::inactivate();
                stateTicks.on(pulseTicks, []{
                    if (mNumber < mPulses) {
                        mState = State::On;        
                    }
                    else {
                        mState = State::Inactive;
                        mNumber = 0;
                    }
                });
                break;
            }
            if (oldstate != mState) {
                stateTicks.reset();
            }
        }
        static inline void blink(uint8_t pulses) {
            mPulses = pulses;
            mNumber = 0;
            mState = State::On;
            stateTicks.reset();
        }
        static inline bool isActive() {
            return mState != State::Inactive;
        }
    private:
        static inline uint8_t mPulses{1};
        static inline uint8_t mNumber{0};
        static inline State mState{State::Inactive};
        inline static External::Tick<Timer> stateTicks;
    };

    template<typename Pin, auto Intervall, std::chrono::milliseconds Pulse, std::chrono::milliseconds Period>
    struct Blinker {
        enum class State : uint8_t {Undefined, Off, Steady, Blink};
        
        static_assert(Pulse <= Period, "wrong parameter");
        
        static inline constexpr auto pulseWidthCount = Pulse / Intervall;
        static inline constexpr auto periodWidthCount = Period / Intervall;
        
//        decltype(pulseWidthCount)::_;
        
        using value_type = etl::typeForValue_t<periodWidthCount>;
//        value_type::_;
        
        static inline constexpr auto maxDisplayablePulses = periodWidthCount / (2 * pulseWidthCount);
        
        using count_type = etl::uint_ranged<uint8_t, 0, maxDisplayablePulses>;
        
        static inline void init() {
            Pin::init();
        }
        static inline void off() {
            mState = State::Off;
        }
        static inline void steady() {
            mState = State::Steady;
        }
        static inline void blink(count_type count) {
            mState = State::Blink;
            mBlinkCount = count;
            mStateCounter.setToBottom();
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
                    const auto phase = mStateCounter % (2 * pulseWidthCount);
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
                ++mStateCounter;
                break;
            default:
                break;
            }
        }
    private:
        static inline bool shouldBlink() {
            return mStateCounter < (2 * pulseWidthCount * mBlinkCount);
        }
        static inline State mState = State::Off;
        static inline etl::uint_ranged_circular<value_type, 0, periodWidthCount + 1> mStateCounter{};
        static inline etl::uint_ranged<uint8_t, 0, maxDisplayablePulses> mBlinkCount{};
    };

    template<typename Pin, typename Timer, std::chrono::milliseconds Pulse, std::chrono::milliseconds Period, uint8_t maxBlinks = 255>
    struct Blinker2 {
        enum class State : uint8_t {Undefined, Off, Steady, Blink, OnePeriodBlink};
        
        static_assert(Pulse <= Period, "wrong parameter");
        
        static inline constexpr auto intervall = Timer::intervall;
        
        static inline constexpr auto pulseWidthCount = Pulse / intervall;
        static inline constexpr auto periodWidthCount = Period / intervall;
        
//        decltype(pulseWidthCount)::_;
        
        using value_type = etl::typeForValue_t<periodWidthCount>;
//        value_type::_;
        
        static inline constexpr auto maxDisplayablePulses = std::min<uint8_t>(maxBlinks, (periodWidthCount / (2 * pulseWidthCount)));
        
        using count_type = etl::uint_ranged<uint8_t, 1, maxDisplayablePulses>;

        using safe_count_type = etl::uint_ranged<uint8_t, 1, maxDisplayablePulses>;
        
        static inline void init() {
            Pin::init();
        }
        static inline void off() {
            mState = State::Off;
        }
        static inline auto blinkCount() {
            return mBlinkCount;
        }
        static inline void steady() {
            mState = State::Steady;
        }
        static inline void blink(count_type count) {
            mState = State::Blink;
            mBlinkCount = count;
            mStateCounter.setToBottom();
        }
        static inline void onePeriod(count_type count) {
            mState = State::OnePeriodBlink;
            mBlinkCount = count;
            mStateCounter.setToBottom();
        }
        static inline bool isActive() {
            return mState != State::Off;
        }
        static inline void ratePeriodic() {
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
                    const auto phase = mStateCounter % (2 * pulseWidthCount);
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
                ++mStateCounter;
                break;
            case State::OnePeriodBlink:
                if (shouldBlink()) {
                    const auto phase = mStateCounter % (2 * pulseWidthCount);
                    if (phase < pulseWidthCount) {
                        Pin::activate();
                    }
                    else {
                        Pin::inactivate();
                    }
                }
                else {
                    mState = State::Off;
                }
                ++mStateCounter;
                break;
            default:
                break;
            }
        }
    private:
        static inline bool shouldBlink() {
            return mStateCounter < (2 * pulseWidthCount * mBlinkCount);
        }
        static inline State mState = State::Off;
        static inline etl::uint_ranged_circular<value_type, 0, periodWidthCount + 1> mStateCounter{};
        static inline count_type mBlinkCount{};
    };
}
