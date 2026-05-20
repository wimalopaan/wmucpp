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

#include "mcu/eeprom.h"
#include "meta.h"
#include "eeprom.h"
#include "etl/event.h"
#include "etl/state.h"
#include "rc/rc_2.h"
#include "rc/crsf_2.h"

template<typename Config>
struct ServoCallback {
    using systemTimer = Config::systemTimer;
    using srv = Config::srv;
    using crsf_cb = Config::crsf_cb;
    using debug = Config::debug;
    
    static inline constexpr External::Tick<systemTimer> stallTicks{300ms};
    
    enum class State : uint8_t {
        None, Idle, Started, Slowed, Stalled
    };
    
    enum class Event : uint8_t {
        None, Started, Stopped, Slowed, OnTarget
    };
    
    static inline void onPing() {
        IO::outl<debug>("# SrvCB onPing: ", srv::servoIds().size());
        crsf_cb::makeServoStrings();
    }
    static inline void onStatus() {
        // IO::outl<debug>("# SrvCB on Status: ", servoIndex);
        for(uint8_t i = 0; i < srv::servoIds().size(); ++i) {
            const uint16_t posDiff = srv::absPhiDiff(i);
            const uint16_t speed = std::abs(srv::actualSpeed(i));
            const uint16_t current = srv::actualCurrent(i);
            const uint16_t load = std::abs(srv::actualLoad(i));
            if ((posDiff > 50) && (speed == 0)) {
                IO::outl<debug>("# SrvCB Slowed: ", i, " d: ", posDiff, " s: ", speed, " c: ", current, " l: ", load);
                mEvents[i] = Event::Slowed;
            }
            else if (posDiff < 10) {
                mEvents[i] = Event::OnTarget;                
            }
        }
    }
    static inline void onStart(const uint8_t servo, const uint16_t diff) {
        // IO::outl<debug>("# SrvCB onStart: ", servo, " ", diff);
        if (diff > 5) {
            mEvents[servo] = Event::Started;
        }
    }
    static inline void onStop(const uint8_t servo) {
        // IO::outl<debug>("# SrvCB onStop: ", servo);
        mEvents[servo] = Event::Stopped;
    }
    static inline void ratePeriodic() {
        for(uint8_t s = 0; s < srv::servoIds().size(); ++s) {
            ratePeriodicServo(s);
        }
    }    
private:
    static inline void ratePeriodicServo(const uint8_t servo) {
        auto& stateTick = mStateTicks[servo];
        auto& event = mEvents[servo];
        ++stateTick;
        auto& state = mServoState[servo];
        const auto oldState = state;
        switch(state) {
        case State::None:
            state = State::Idle;
            break;
        case State::Idle:
            event.on(Event::Started, [&]{
               state = State::Started; 
            });
            break;
        case State::Started:
            event.on(Event::Slowed, [&]{
                state = State::Slowed; 
            }).thenOn(Event::Stopped, [&]{
                state = State::Idle;
            });
            break;
        case State::Slowed:
            // event.on(Event::Started, [&]{
            //     state = State::Started;
            // });
            event.on(Event::OnTarget, [&]{
                state = State::Idle; 
            });
            stateTick.on(stallTicks, [&]{
                state = State::Stalled;
            });
            break;
        case State::Stalled:
            event.on(Event::OnTarget, [&]{
                state = State::Idle; 
            });
            break;
        }
        if (oldState != state) {
            stateTick.reset();
            switch(state) {
            case State::None:
                IO::outl<debug>("# SrvCB S[", servo, "] None");
                break;
            case State::Idle:
                IO::outl<debug>("# SrvCB S[", servo, "] Idle");
                break;
            case State::Started:
                IO::outl<debug>("# SrvCB S[", servo, "] Started");
                break;
            case State::Slowed:
                IO::outl<debug>("# SrvCB S[", servo, "] Slowed");
                break;
            case State::Stalled:
                IO::outl<debug>("# SrvCB S[", servo, "] Stalled");
                break;
            }
        }
    }
    static inline std::array<External::Tick<systemTimer>, srv::MaxServos> mStateTicks;
    static inline std::array<etl::SlotEvent<Event>, srv::MaxServos> mEvents{};
    static inline std::array<etl::State<State>, srv::MaxServos> mServoState{};
};
