/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include <limits>
#include <algorithm>
#include <array>
#include <cstring>

#include "mcu/alternate.h"
#include "byte.h"
#include "etl/algorithm.h"
#include "etl/ranged.h"
#include "units.h"
#include "tick.h"
#include "rc/rc_2.h"
#include "debug_pin.h"

namespace External::GPS {
    namespace V2 {
        namespace Protocols {
            struct UBlox {
                static inline uint16_t parse(const uint8_t* const data) {
                    const uint8_t msgClass = data[2];
                    const uint8_t msgId    = data[3];
                    const uint16_t msgLen = data[4] + (data[5] << 8);

                    if ((msgClass == 0x01) && (msgId == 0x07)) {
                        parseNavPvt(data + 6);
                    }
                    else if ((msgClass == 0x10) && (msgId == 0x10)) {
                        parseEsfStatus(data + 6);
                    }
                    return 6 + msgLen + 2; // offset points to next byte after this message
                }
                // private:
                static inline void parseEsfStatus(const uint8_t* const /*data*/) {
                    ++mStatusPackages;
                }
                static inline void parseNavPvt(const uint8_t* const data) {
                    ++mNavPackages;
                    mYear = data[4] + (data[5] << 8);
                    mMonth = data[7];
                    mDay = data[8];
                    mFlags = data[21];
                    mSatCount = data[23];
                    mLongitude = etl::littleEndianI32(&data[24]);
                    mLatitude  = etl::littleEndianI32(&data[28]);
                    mAltitude = etl::littleEndianI32(&data[36]);
                    mSpeed = etl::littleEndianI32(&data[60]); // mm/s
                    mSpeed *= 3600; // mm/h
                    mSpeed /= (1000 * 100); // 1/10 km/h
                    mHeadingM = etl::littleEndianI32(&data[64]) / 1000; // 1/100 degree
                    mHeading = etl::littleEndianI32(&data[84]) / 1000;
                }
                static inline uint8_t mFlags = 0;
                static inline int32_t mLongitude = 0;
                static inline int32_t mLatitude  = 0;
                static inline int32_t mAltitude  = 0;
                static inline int32_t mSpeed = 0;
                static inline int32_t mHeading = 0;
                static inline int32_t mHeadingM = 0;
                static inline uint16_t mYear = 0;
                static inline uint8_t mMonth = 0;
                static inline uint8_t mDay = 0;
                static inline uint8_t mSatCount = 0;
                static inline uint16_t mNavPackages = 0;
                static inline uint16_t mStatusPackages = 0;
            };
            namespace NMEA {
                namespace Util {
                    template<uint8_t NFrac = 1>
                    static inline int32_t toNumber(const char* const data, const char* const end) {
                        int32_t num = 0;
                        uint8_t nFrac = 0;
                        bool frac = false;
                        for(uint8_t i = 0; i < (end - data); ++i) {
                            const char c = data[i];
                            if (std::isdigit(c)) {
                                num *= 10;
                                num += etl::asDigit(c);
                                if (frac) {
                                    ++nFrac;
                                    if (nFrac >= NFrac) {
                                        break;
                                    }
                                }
                            }
                            else if (c == '.') {
                                frac = true;
                            }
                        }
                        return num;
                    }
                    template<bool Longitude = false>
                    static inline int32_t toCrsfDegree(const char* const data, const char* const end) {
                        static constexpr uint8_t offset = Longitude ? 1 : 0;
                        const int32_t deg = [&]{
                            if constexpr(Longitude) {
                                return etl::asDigit(data[0]) * 100 + etl::asDigit(data[1]) * 10 + etl::asDigit(data[2]);
                            }
                            else {
                                return etl::asDigit(data[0]) * 10 + etl::asDigit(data[1]);
                            }
                        }();
                        int32_t min = etl::asDigit(data[2 + offset]) * 10 + etl::asDigit(data[3 + offset]);;
                        uint8_t nMinFrac = 0;
                        if (data[4 + offset] == '.') {
                            for(uint8_t i = 0; i < (end - &data[5 + offset]); ++i) {
                                ++nMinFrac;
                                min *= 10;
                                min += etl::asDigit(data[5 + i + offset]);
                            }
                        }
                        int32_t deziLat = (min * 100) / 60;
                        for(uint8_t i = 0; i < (5 - nMinFrac); ++i) {
                            deziLat *= 10;
                        }
                        return deg * 10'000'000L + deziLat;
                    }
                }
                struct RMC {
                    static inline void parseFields(const char* const data, const char* const end, const uint8_t field) {
                        if (field == 1) {
                            for(uint8_t i = 0; (i < (end - data)) && (i < mTime.size()); ++i) {
                                mTime[i] = data[i];
                            }
                        }
                        else if (field == 2) {
                            if (data[0] == 'A') {
                                mTimeValid = true;
                            }
                            else {
                                mTimeValid = false;
                            }
                        }
                        else if (field == 3) {
                            mLatitude = Util::toCrsfDegree<false>(data, end);
                        }
                        else if (field == 4) {
                            if (data[0] == 'N') {
                                mLatNorth = true;
                            }
                            else if (data[0] == 'S') {
                                mLatNorth = false;
                            }
                        }
                        else if (field == 5) {
                            mLongitude = Util::toCrsfDegree<true>(data, end);
                        }
                        else if (field == 5) {
                            if (data[0] == 'W') {
                                mLonWest = true;
                            }
                            else if (data[0] == 'E') {
                                mLonWest = false;
                            }
                        }
                        else if (field == 9) {
                            for(uint8_t i = 0; (i < (end - data)) && (i < mDate.size()); ++i) {
                                mDate[i] = data[i];
                            }
                        }
                    }
                    static inline std::array<char, 6> mTime{};
                    static inline std::array<char, 6> mDate{};
                    static inline bool mTimeValid = false;
                    static inline int32_t mLatitude = 0;
                    static inline bool mLatNorth = true;
                    static inline int32_t mLongitude = 0;
                    static inline bool mLonWest = true;
                };
                struct VTG {
                    static inline void parseFields(const char* const data, const char* const end, const uint8_t field) {
                        if (field == 1) {
                            mHeading = Util::toNumber<2>(data, end);
                        }
                        else if (field == 7) {
                            mSpeed = Util::toNumber<1>(data, end);
                        }
                    }
                    static inline int32_t mHeading = 0;
                    static inline int16_t mSpeed = 0;
                };
                struct GSV {
                    static inline void parseFields(const char* const data, const char* const end, const uint8_t field) {
                        if (field == 3) {
                            mSatCount = Util::toNumber<0>(data, end);
                        }
                    }
                    static inline uint8_t mSatCount = 0;
                };
                static inline uint16_t skipSentence(const auto& data, const uint16_t offset) {
                    uint16_t i = offset;
                    while(i < data.size()) {
                        if (std::strncmp((const char*)&data[i], "\r\n", 2) == 0) {
                            return i + 2;
                        }
                        ++i;
                    }
                    return i;
                }
            }
        }
        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct Input {
            using clock = Config::clock;
            using systemTimer = Config::systemTimer;
            using debug = Config::debug;
            using pin = Config::pin;
            using tp = Config::tp;

            using ublox = Protocols::UBlox;

            using rmc = Protocols::NMEA::RMC;
            using vtg = Protocols::NMEA::VTG;
            using gsv = Protocols::NMEA::GSV;

            static inline constexpr std::array<uint32_t, 2> baudrates{9600, 115200};

            static inline void nextBaudrate() {
                if (!mActive) return;
                static uint8_t index = 0;
                ++index;
                if (index >= baudrates.size()) {
                    index = 0;
                }
                IO::outl<debug>("# GPS next baudrate: ", baudrates[index]);
                uart::baud(baudrates[index]);
            }

            struct UartConfig {
                using Clock = clock;
                using ValueType = uint8_t;
                static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::RxOnly;
                static inline constexpr uint32_t baudrate = baudrates[0];
                using DmaChComponent = Config::dmaChComponent;
                struct Rx {
                    static inline constexpr bool enable = true;
                    static inline constexpr size_t size = 1024;
                    static inline constexpr size_t idleMinSize = 4;
                };
                struct Isr {
                    static inline constexpr bool idle = true;
                };
                using tp = Config::tp;
            };
            using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;
            static inline void init() {
                static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::RX>;
                IO::outl<debug>("# GPS init");
                Mcu::Arm::Atomic::access([]{
                    mState = State::Init;
                    mPackagesCount = 0;
                    mEvent = Event::None;
                    mActive = true;
                    uart::init();
                });
                pin::afunction(af);
                pin::template pullup<true>();
            }
            static inline void reset() {
                IO::outl<debug>("# GPS reset");
                Mcu::Arm::Atomic::access([]{
                    uart::reset();
                    mActive = false;
                });
                pin::analog();
            }
            enum class State : uint8_t {Init, Run};
            enum class Event : uint8_t {None, ReceiveComplete};
            static inline void event(const Event e) {
                mEvent = e;
            }
            static inline void periodic() {
                switch(mState) {
                case State::Init:
                    break;
                case State::Run:
                    if (mEvent.is(Event::ReceiveComplete)) {
                        readReply();
                    }
                    break;
                }
            }
            static inline constexpr External::Tick<systemTimer> initTicks{1000ms};
            static inline void ratePeriodic() {
                const auto oldState = mState;
                ++mStateTick;
                switch(mState) {
                case State::Init:
                    mStateTick.on(initTicks, []{
                        mState = State::Run;
                    });
                    break;
                case State::Run:
                    break;
                }
                if (oldState != mState) {
                    mStateTick.reset();
                    switch(mState) {
                    case State::Init:
                        break;
                    case State::Run:
                        IO::outl<debug>("# GPS Run");
                        uart::template rxEnable<true>();
                        break;
                    }
                }
            }
            static inline void update() {}
            struct Isr {
                static inline void onIdle(const auto f) {
                    if (mActive) {
                        const auto f2 = [&](const volatile uint8_t* const data, const uint16_t size){
                            f();
                            if (validityCheck(data, size)) {
                                event(Event::ReceiveComplete);
                                return true;
                            }
                            return false;
                        };
                        uart::Isr::onIdle(f2);
                    }
                }
            };
            static inline uint16_t packages() {
                const uint16_t p = mPackagesCount;
                mPackagesCount = 0;
                return p;
            }
            private:
            static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t n) {
                if (data[0] == '$') { // NMEA
                    if (data[1] != 'G') {
                        return false;
                    }
                    if (data[n - 2] != '\r') {
                        return false;
                    }
                    if (data[n - 1] != '\n') {
                        return false;
                    }
                    return true;
                }
                else {
                    if (data[0] == 0xb5) {
                        if (data[1] == 0x62) { // UBLOX binary followed by NMEA
                            if (data[n - 2] != '\r') {
                                return false;
                            }
                            if (data[n - 1] != '\n') {
                                return false;
                            }
                            return true;
                        }
                    }
                    return false;
                }
            }
            template<typename P>
            static inline uint16_t analyze(const auto& data, const uint16_t offset) {
                uint8_t field = 1;
                for(uint16_t i = offset; i < data.size(); ) {
                    if (data[i] == ',') {
                        ++field;
                        i = i + 1;
                    }
                    else {
                        const char* fBegin = (const char*)&data[i];
                        for(uint16_t n = i + 1; n < data.size(); ++n) {
                            if (data[n] == ',') {
                                data[n] = '\0';
                                P::parseFields(fBegin, (const char*)&data[n], field);
                                ++field;
                                i = n + 1;
                                break;
                            }
                            else if (data[n] == '*') {
                                ++mPackagesCount;
                                return n + 5;
                            }
                            else {
                            }
                        }
                    }
                }
                return data.size();
            }
            static inline void readReply() {
                uart::readBuffer([](const auto& data){
                    ++mPackagesCount;
                    uint16_t i = 0;
                    while(i < data.size()) {
                        if (data[i] == '$') {
                            if (std::strncmp((const char*)&data[i + 3], "RMC", 3) == 0) {
                                i = analyze<rmc>(data, i + 7);
                            }
                            else if (std::strncmp((const char*)&data[i + 3], "VTG", 3) == 0) {
                                i = analyze<vtg>(data, i + 7);
                            }
                            else if (std::strncmp((const char*)&data[i + 3], "GSV", 3) == 0) {
                                i = analyze<gsv>(data, i + 7);
                            }
                            else {
                                i = Protocols::NMEA::skipSentence(data, i);
                            }
                        }
                        else if ((data[i] == 0xb5) && (data[i + 1] == 0x62)) {
                            i += ublox::parse((const uint8_t*)&data[i]);
                        }
                        else {
                            ++i;
                        }
                    }
                });
            }
            static inline volatile bool mActive = false;
            static inline std::array<uint16_t, RC::Protokoll::IBus::V2::numberOfChannels> mChannels; // sbus
            static inline uint16_t mPackagesCount = 0;
            static inline volatile etl::Event<Event> mEvent;
            static inline volatile State mState = State::Init;
            static inline External::Tick<systemTimer> mStateTick;
        };
    }
}
