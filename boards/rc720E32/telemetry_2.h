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

template<typename Buffer, typename Storage, typename Servos, typename Escs, typename SystemTimer, typename Debug>
struct Telemetry {
    using debug = Debug;
    using systemTimer = SystemTimer;
    using buffer = Buffer;
    using storage = Storage;
    using servos = Servos;
    using escs = Escs;

    static inline constexpr uint8_t infoRate = 100; // out of many telem. packets an info packet (ArduPilot-Tunnel) is send
    using tick_t = External::Tick<systemTimer, uint32_t>;
    static inline constexpr tick_t autoOnTicks{30'000ms};

    enum class State : uint8_t {On, TempOff};
    enum class Event : uint8_t {None, OffWithAutoOn};

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
        ++mFrameCounter;
        if (mFrameCounter < infoRate) {
            buffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::ArduPilot, [&](auto& d){
                d.push_back(RC::Protokoll::Crsf::V4::Address::Handset);
                d.push_back((uint8_t)storage::eeprom.address);
                d.push_back(ArduPilotTunnel::Schottel::AppId);
                d.push_back(ArduPilotTunnel::Schottel::Type::CombinedTelemetry); // packet type
                d.push_back(mValues);
                d.push_back(mTurns);
                d.push_back(mFlags);
            });
        }
        else {
            mFrameCounter = 0;
            buffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::ArduPilot, [&](auto& d){
                d.push_back(RC::Protokoll::Crsf::V4::Address::Handset);
                d.push_back((uint8_t)storage::eeprom.address);
                d.push_back(ArduPilotTunnel::Schottel::AppId);
                d.push_back(ArduPilotTunnel::Schottel::Type::DeviceInfo); // packet type
                d.push_back(servos::fwVersion(0));
                d.push_back(servos::hwVersion(0));
                d.push_back(servos::fwVersion(1));
                d.push_back(servos::hwVersion(1));

                d.push_back(escs::fwVersion(0));
                d.push_back(escs::hwVersion(0));
                d.push_back(escs::fwVersion(1));
                d.push_back(escs::hwVersion(1));

                d.push_back((uint8_t)SW_VERSION);
                d.push_back((uint8_t)HW_VERSION);
            });
        }
    }
    template<auto N>
    static inline void phi(const uint16_t p) {
        mValues[5 * N] = p;
    }
    template<auto N>
    static inline void amp(const uint16_t a) {
        mValues[5 * N + 1] = a;
    }
    static inline void actual(const uint8_t n, const uint16_t a) {
        mValues[5 * n + 2] = a;
    }
    static inline void current(const uint8_t n, const uint16_t c) {
        mValues[5 * n + 3] = c;
    }
    static inline void rpm(const uint8_t n, const uint16_t r) {
        mValues[5 * n + 4] = r;
    }
    static inline void turns(const uint8_t n, const int8_t t) {
        mTurns[n] = t;
    }
    static inline void alarm(const uint8_t n, const bool a) {
        if (a) {
            mFlags |= (0x01 << n);
        }
        else {
            mFlags &= ~(0x01 << n);
        }
    }
    private:
    static inline uint8_t mFrameCounter = 0;
    static inline std::array<uint16_t, 10> mValues{};
    static inline std::array<int8_t, 2> mTurns{};
    static inline uint8_t mFlags = 0;
    static inline tick_t mStateTick;
    static inline etl::Event<Event> mEvent;
    static inline State mState = State::On;
};

