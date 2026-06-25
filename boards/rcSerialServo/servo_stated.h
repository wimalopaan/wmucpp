/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "meta.h"
#include "eeprom.h"
#include "tick.h"
#include "etl/event.h"
#include "etl/state.h"
#include "rc/rc_2.h"

template<typename Servo, typename PA, typename Timer, typename Debug>
struct ServoStated {
    using srv = Servo;
    using pa = typename PA::input;
    using debug = Debug;
    using systemTimer = Timer;
    
    static inline constexpr External::Tick<systemTimer> incTicks{100ms};
    
    static inline constexpr uint16_t threshLow  = RC::Protokoll::Crsf::V4::min + 100;
    static inline constexpr uint16_t threshHigh = RC::Protokoll::Crsf::V4::max - 100;
    static inline constexpr uint16_t neutralLow = RC::Protokoll::Crsf::V4::mid - 100;
    static inline constexpr uint16_t neutralHigh= RC::Protokoll::Crsf::V4::mid + 100;
    
    enum class State : uint8_t {
        None = 0, WaitNeutral, Lower, Higher, LowFinished
    };
    enum class Event : uint8_t {
        None, Start
    };

    static inline void reset(const uint8_t srv) {
        IO::outl<debug>("SrvStated reset: ", srv);
        mSrv = srv;
        mEvent = Event::Start;
    }
    static inline uint8_t state() {
        IO::outl<debug>("SrvStated state: ", mState.toInt());
        return mState.toInt();
    }
    static inline bool isFinished() {
        return (mState == State::LowFinished) || (mState == State::None);
    }
    static inline void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::None:
            mEvent.on(Event::Start, []{
                mState = State::WaitNeutral;
            });
            break;
        case State::WaitNeutral:
            if (const auto v = pa::value(mSrv); (v >= neutralLow) && (v <= neutralHigh)) {
                mState = State::Lower;
            }
            break;
        case State::Lower:
            if (const auto v = pa::value(mSrv); v < threshLow) {
                mStateTick.on(incTicks, [&]{
                    mActualPos -= (threshLow - v);
                    srv::setRaw(mSrv, mActualPos);
                    IO::outl<debug>("SrvStated l: ", mSrv, " p: ", mActualPos);                   
                });
            }
            else if (v > threshHigh) {
                mState = State::Higher;
            }
            break;
        case State::Higher:
            if (const auto v = pa::value(mSrv); v > threshHigh) {
                mStateTick.on(incTicks, [&]{
                    mActualPos += (v - threshHigh);
                    srv::setRaw(mSrv, mActualPos);
                    IO::outl<debug>("SrvStated h: ", mSrv, " p: ", mActualPos);                   
                });
            }
            else if (v < threshLow) {
                mState = State::LowFinished;
            }
            break;
        case State::LowFinished:
            mState = State::None;
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::None:
                IO::outl<debug>("SrvStated S none");                                        
                break;
            case State::WaitNeutral:
                IO::outl<debug>("SrvStated S wait");                                        
                srv::enable(mSrv, false);
                break;
            case State::Lower:
                IO::outl<debug>("SrvStated S lower");                                        
                mActualPos = srv::actualPos(mSrv);
                break;
            case State::Higher:
                IO::outl<debug>("SrvStated S higher");                                        
                mLowPos = mActualPos;
                break;
            case State::LowFinished:
                IO::outl<debug>("SrvStated S finished");                                        
                mHighPos = mActualPos;
                if (mHighPos > mLowPos) {
                    srv::gearLowHigh(mSrv, mLowPos, mHighPos);
                    srv::enable(mSrv, true);
                }
                mActualPos = RC::Protokoll::Crsf::V4::mid;
                break;
            }           
        }
    }
private:
    static inline uint8_t mSrv = 0;
    static inline int16_t mLowPos = 0;
    static inline int16_t mHighPos = 0;
    static inline int16_t mActualPos = 0;
    static inline External::Tick<systemTimer> mStateTick;
    static inline etl::SlotEvent<Event> mEvent;
    static inline etl::State<State> mState;
};
