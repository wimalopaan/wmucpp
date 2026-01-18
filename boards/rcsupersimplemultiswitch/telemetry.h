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

#include <array>

#include "messagebuilder.h"

template<typename Config>
struct Telemetry {
    using debug = Config::debug;
    using crsf = Config::crsf;

    static inline void gotLink() {
        switch(mCounter) {
        case 0:
            sendBits();
            ++mCounter;
            break;
        case 1:
            sendTemperature();
            ++mCounter;
            break;
        case 2:
            sendVoltage();
            mCounter = 0;
            break;
        }
    }
    static inline void temperature(const int16_t t) {
        mTemp = t;
    }
    static inline void voltage(const uint16_t v) {
        mVoltage = v;
    }
    static inline void status(const uint8_t b) {
        mBits = b;
    }
    static inline void address(const uint8_t a) {
        mAddress = a;
    }
    private:
    static inline void sendTemperature() {
        etl::outl<debug>("send temp"_pgm);
        MessageBuilder<crsf> b(mMessage, Crsf::Type::Temp);
        b.push_back(mId);
        b.push_back(mTemp);
    }
    static inline void sendBits() {
        etl::outl<debug>("send bits"_pgm);
        MessageBuilder<crsf> b(mMessage, Crsf::Type::PassThru);
        b.push_back(Crsf::Address::Handset);
        b.push_back(Crsf::Address::Controller);
        b.push_back(Crsf::PassThru::SubType::Switch);
        b.push_back(Crsf::PassThru::AppId::Status);
        b.push_back(mAddress);
        b.push_back(mBits);
    }
    static inline void sendVoltage() {
        etl::outl<debug>("send voltage"_pgm);
        MessageBuilder<crsf> b(mMessage, Crsf::Type::Cells);
        b.push_back(mId);
        b.push_back(mVoltage);
    }
    static inline uint8_t mCounter{};
    static inline std::array<std::byte, Crsf::maxMessageSize> mMessage{};
    static inline uint8_t mId{0};
    static inline int16_t mTemp{0};
    static inline uint16_t mVoltage{0};
    static inline uint16_t mAppId{6000};
    static inline uint16_t mSubType{0xa0};
    static inline uint8_t mAddress{0};
    static inline uint8_t mBits{0};
};
