/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "etl/algorithm.h"
#include "rc/crsf_2.h"
#include "tick.h"
#include "output.h"
#include "units.h"

using namespace std::literals::chrono_literals;

template<typename Config>
struct Telemetry {
    using debug = Config::debug;
    using systemTimer = Config::timer;
    using buffer = Config::messagebuffer;
    using storage = Config::storage;

    using tick_t = External::Tick<systemTimer, uint32_t>;
    static inline constexpr tick_t autoOnTicks{30'000ms};

    enum class State : uint8_t {On, TempOff};
    enum class Event : uint8_t {None, OffWithAutoOn};

    static inline void voltage(const uint16_t v) {
        mVoltage = v;
    }
    static inline void temp(const int16_t t) {
        mTemp = t;
    }

    static inline void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::On:
            mEvent.on(Event::OffWithAutoOn, []{
                mState = State::TempOff;
            });
            break;
        case State::TempOff:
            mEvent.on(Event::OffWithAutoOn, []{
               mStateTick.reset();
            });
            mStateTick.on(autoOnTicks, []{
                mState = State::On;
            });
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::On:
                IO::outl<debug>("# Telemetry: On");
                break;
            case State::TempOff:
                IO::outl<debug>("# Telemetry: TempOff");
                break;
            }
        }
    }

    static inline void event(const Event e) {
        mEvent = e;
    }
    static inline void disableWithAutoOn() {
        event(Event::OffWithAutoOn);
    }

    static inline void push(const uint8_t type, auto f) {
        if (mState != State::On) return;
        buffer::create_back(type, [&](auto& d){
            f(d);
        });
    }
    static inline void next() {
        if (mState != State::On) return;
        using namespace RC::Protokoll::Crsf::V4;
        // buffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::Battery, [&](auto& d){
        //     d.push_back(mVoltage);
        //     d.push_back(uint16_t(0));
        //     d.push_back(uint8_t(0));
        //     d.push_back(uint8_t(0));
        //     d.push_back(uint8_t(0));
        //     d.push_back(uint8_t(0));
        // });
        buffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::Temp, [&](auto& d){
            d.push_back(uint8_t(storage::eeprom.temp_id));
            d.push_back(mTemp);
        });
        buffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::Cells, [&](auto& d){
            d.push_back(uint8_t(storage::eeprom.cells_id));
            d.push_back(mVoltage);
        });
    }
    private:
    static inline uint16_t mVoltage{};
    static inline int16_t mTemp{};
    static inline uint8_t mFlags = 0;
    static inline tick_t mStateTick;
    static inline etl::Event<Event> mEvent;
    static inline State mState = State::On;
};

