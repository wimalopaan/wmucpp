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

template<typename Buffer, typename Storage, typename Servos, typename Escs, typename Debug>
struct Telemetry {
    using debug = Debug;
    using buffer = Buffer;
    using storage = Storage;
    using servos = Servos;
    using escs = Escs;

    static inline constexpr uint8_t infoRate = 100;

    static inline void push(const uint8_t type, auto f) {
        buffer::create_back(type, [&](auto& d){
            f(d);
        });
    }
    static inline void next() {
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
    // static inline uint8_t mCounter = 0;
};

