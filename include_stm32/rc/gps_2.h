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
            struct CheckSum {
                uint8_t operator+=(const uint8_t b) {
                    ck_a = ck_a + b;
                    ck_b = ck_b + ck_a;
                    return b;
                }
                uint8_t ck_a = 0;
                uint8_t ck_b = 0;
            };
            struct UBlox {
                static inline uint16_t parse(const uint8_t* const data) {
                    const uint8_t msgClass = data[2];
                    const uint8_t msgId    = data[3];
                    const uint16_t msgLen = data[4] + (data[5] << 8);
                    CheckSum cs;
                    for(uint16_t i = 2; i < (msgLen + 6); ++i) {
                        cs += data[i];
                    }
                    if ((cs.ck_a == data[msgLen + 6]) && (cs.ck_b == data[msgLen + 7])) {
                        if ((msgClass == 0x01) && (msgId == 0x07)) {
                            parseNavPvt(data + 6);
                        }
                        else if ((msgClass == 0x10) && (msgId == 0x10)) {
                            parseEsfStatus(data + 6);
                        }
                        else if ((msgClass == 0x01) && (msgId == 0x04)) { // DOP
                            parseDOP(data + 6);
                        }
                        else if ((msgClass == 0x0a) && (msgId == 0x04)) { // Version
                            parseVersion(data + 6, msgLen);
                        }
                        else if ((msgClass == 0x05) && (msgId == 0x01)) { // Ack
                            parseAck(data + 6);
                        }
                        else if ((msgClass == 0x05) && (msgId == 0x00)) { // NAck
                            parseNAck(data + 6);
                        }
                    }
                    return 6 + msgLen + 2; // offset points to next byte after this message
                }
                // private:
                static inline void parseAck(const uint8_t* const /*data*/) {
                    ++mAckPackages;
                }
                static inline void parseNAck(const uint8_t* const /*data*/) {
                    ++mNAckPackages;
                }
                static inline void parseDOP(const uint8_t* const data) {
                    ++mDopPackages;
                    mGDop = data[4] + (data[5] << 8);
                    mPDop = data[6] + (data[7] << 8);
                    mHDop = data[12] + (data[13] << 8);
                }
                static inline void parseVersion(const uint8_t* const data, const uint16_t len) {
                    ++mVerPackages;
                    const uint8_t nExtensions = (len - 40) / 30;
                    uint8_t k = 0;
                    for(uint16_t i = 0; i < mSWVersion.size(); ++i) {
                        mSWVersion[k++] = data[i];
                    }
                    k = 0;
                    for(uint16_t i = 30; i < (30 + mHWVersion.size()); ++i) {
                        mHWVersion[k++] = data[i];
                    }
                    for(uint8_t e = 0; (e < nExtensions) && (e < mExtensions.size()); ++e) {
                        k = 0;
                        for(uint16_t i = 40 + (e * 30); i < (40 + (e + 1) * mExtensions[0].size()); ++i) {
                            mExtensions[e][k++] = data[i];
                        }
                    }
                }
                static inline void parseEsfStatus(const uint8_t* const /*data*/) {
                    ++mStatusPackages;
                }
                static inline void parseNavPvt(const uint8_t* const data) {
                    ++mNavPackages;
                    mYear = data[4] + (data[5] << 8);
                    mMonth = data[7];
                    mDay = data[8];
                    mFix = data[20];
                    mFlags = data[21];
                    mSatCount = data[23];
                    mLongitude = etl::littleEndianI32(&data[24]);
                    mLatitude  = etl::littleEndianI32(&data[28]);
                    mAltitude = etl::littleEndianI32(&data[32]);
                    mHMSL = etl::littleEndianI32(&data[36]);
                    mHAcc = etl::littleEndianU32(&data[40]);
                    mSpeed = etl::littleEndianI32(&data[60]); // mm/s
                    mSpeed *= 3600; // mm/h
                    mSpeed /= (1000 * 100); // 1/10 km/h
                    mHeadingM = etl::littleEndianI32(&data[64]) / 1000; // 1/100 degree
                    mHeading = etl::littleEndianI32(&data[84]) / 1000;
                }
                static inline std::array<char, 30> mSWVersion;
                static inline std::array<char, 10> mHWVersion;
                static inline std::array<std::array<char, 30>, 10> mExtensions;
                static inline uint16_t mGDop = 0;
                static inline uint16_t mPDop = 0;
                static inline uint16_t mHDop = 0;
                static inline uint8_t mFix = 0;
                static inline uint8_t mFlags = 0;
                static inline int32_t mLongitude = 0;
                static inline int32_t mLatitude  = 0;
                static inline int32_t mAltitude  = 0;
                static inline int32_t mHMSL  = 0;
                static inline uint32_t mHAcc  = 0;
                static inline int32_t mSpeed = 0;
                static inline int32_t mHeading = 0;
                static inline int32_t mHeadingM = 0;
                static inline uint16_t mYear = 0;
                static inline uint8_t mMonth = 0;
                static inline uint8_t mDay = 0;
                static inline uint8_t mSatCount = 0;
                static inline uint8_t mDopPackages = 0;
                static inline uint16_t mNavPackages = 0;
                static inline uint16_t mStatusPackages = 0;
                static inline uint16_t mVerPackages = 0;
                static inline uint16_t mAckPackages = 0;
                static inline uint16_t mNAckPackages = 0;
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
            using pin_rx = Config::pin_rx;
            using pin_tx = Config::pin_tx;
            using tp = Config::tp;

            using ublox = Protocols::UBlox;

            using rmc = Protocols::NMEA::RMC;
            using vtg = Protocols::NMEA::VTG;
            using gsv = Protocols::NMEA::GSV;

            static inline constexpr std::array<uint32_t, 2> baudrates{115200, 9600};

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
                static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::HalfDuplex;
                static inline constexpr bool RxTxLinesDifferent = !std::is_same_v<pin_rx, pin_tx>;
                static inline constexpr uint32_t baudrate = baudrates[0];
                using DmaChComponent = Config::dmaChComponent;
                struct Rx {
                    static inline constexpr bool enable = false;
                    static inline constexpr size_t size = 1024;
                    static inline constexpr size_t idleMinSize = 4;
                };
                struct Tx {
                    static inline constexpr bool enable = true;
                    static inline constexpr bool singleBuffer = true;
                    static inline constexpr size_t size = 16;
                };
                struct Isr {
                    static inline constexpr bool idle = true;
                    static inline constexpr bool txComplete = true;
                };
                using tp = Config::tp;
            };
            using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;
            static inline void init() {
                static constexpr uint8_t afrx = Mcu::Stm::AlternateFunctions::mapper_v<pin_rx, uart, Mcu::Stm::AlternateFunctions::RX>;
                static constexpr uint8_t aftx = Mcu::Stm::AlternateFunctions::mapper_v<pin_tx, uart, Mcu::Stm::AlternateFunctions::TX>;
                IO::outl<debug>("# GPS init");
                Mcu::Arm::Atomic::access([]{
                    mState = State::Init;
                    mPackagesCount = 0;
                    mEvent = Event::None;
                    mActive = true;
                    uart::init();
                });
                pin_rx::afunction(afrx);
                pin_rx::template pullup<true>();
                pin_tx::afunction(aftx);
            }
            static inline void reset() {
                IO::outl<debug>("# GPS reset");
                Mcu::Arm::Atomic::access([]{
                    uart::reset();
                    mActive = false;
                });
                pin_rx::analog();
                pin_tx::analog();
            }
            enum class State : uint8_t {Init,
                                        SetMessageInterval1,
                                        SetMessageInterval2,
                                        SetMessageInterval3,
                                        SetMessageInterval3N,
                                        GetVersion, Run};
            enum class Event : uint8_t {None, ReceiveComplete, TransferComplete};

            static inline constexpr std::array<uint8_t, 18> disableNMEA{0x0a, 0x44, 0x09, 0x00,
                                                                    0x01, 0x43, 0x42, 0x0d,
                                                                    0x40, 0x06, 0x02, 0x07,
                                                                    0x03, 0x0e, 0x41, 0x0f,
                                                                    0x05, 0x08};
            static inline uint8_t mDisCounter = 0;

            static inline void event(const Event e) {
                mEvent = e;
            }
            static inline void periodic() {
                if (mEvent.is(Event::ReceiveComplete)) {
                    readReply();
                }
            }
            static inline constexpr External::Tick<systemTimer> initTicks{1000ms};
            static inline void ratePeriodic() {
                const auto oldState = mState;
                ++mStateTick;
                switch(mState) {
                case State::Init:
                    mStateTick.on(initTicks, []{
                        mState = State::SetMessageInterval1;
                    });
                    break;
                case State::SetMessageInterval1:
                    if (mEvent.is(Event::TransferComplete)) {
                        mState = State::SetMessageInterval2;
                    }
                    break;
                case State::SetMessageInterval2:
                    if (mEvent.is(Event::TransferComplete)) {
                        mState = State::SetMessageInterval3;
                        mDisCounter = 0;
                    }
                    break;
                case State::SetMessageInterval3:
                    if (mEvent.is(Event::TransferComplete)) {
                        if (++mDisCounter < disableNMEA.size()) {
                            mState = State::SetMessageInterval3N;
                        }
                        else {
                            mState = State::GetVersion;
                        }
                    }
                    break;
                case State::SetMessageInterval3N:
                    mState = State::SetMessageInterval3;
                    break;
                case State::GetVersion:
                    if (mEvent.is(Event::TransferComplete)) {
                        mState = State::Run;
                    }
                    break;
                case State::Run:
                    break;
                }
                if (oldState != mState) {
                    mStateTick.reset();
                    switch(mState) {
                    case State::Init:
                        IO::outl<debug>("# GPS Run");
                        break;
                    case State::SetMessageInterval1:
                        IO::outl<debug>("# GPS SM1");
                        mEvent = Event::None;
                        uart::fillSendBuffer([](auto& data){
                            Protocols::CheckSum cs;
                            uint8_t i = 0;
                            data[i++] = 0xb5;
                            data[i++] = 0x62;
                            data[i++] = (cs += 0x06);
                            data[i++] = (cs += 0x01);
                            const uint16_t l = 3;
                            data[i++] = (cs += (l & 0xff));
                            data[i++] = (cs += ((l >> 8) & 0xff));
                            data[i++] = (cs += 0x01);
                            data[i++] = (cs += 0x07);
                            data[i++] = (cs += 1);
                            data[i++] = cs.ck_a;
                            data[i++] = cs.ck_b;
                            return i;
                        });
                        break;
                    case State::SetMessageInterval2:
                        IO::outl<debug>("# GPS SM2");
                        mEvent = Event::None;
                        uart::fillSendBuffer([](auto& data){
                            Protocols::CheckSum cs;
                            uint8_t i = 0;
                            data[i++] = 0xb5;
                            data[i++] = 0x62;
                            data[i++] = (cs += 0x06);
                            data[i++] = (cs += 0x01);
                            const uint16_t l = 3;
                            data[i++] = (cs += (l & 0xff));
                            data[i++] = (cs += ((l >> 8) & 0xff));
                            data[i++] = (cs += 0x01);
                            data[i++] = (cs += 0x04);
                            data[i++] = (cs += 1);
                            data[i++] = cs.ck_a;
                            data[i++] = cs.ck_b;
                            return i;
                        });
                        break;
                    case State::SetMessageInterval3:
                        IO::outl<debug>("# GPS SM3");
                        mEvent = Event::None;
                        uart::fillSendBuffer([](auto& data){
                            Protocols::CheckSum cs;
                            uint8_t i = 0;
                            data[i++] = 0xb5;
                            data[i++] = 0x62;
                            data[i++] = (cs += 0x06);
                            data[i++] = (cs += 0x01);
                            const uint16_t l = 3;
                            data[i++] = (cs += (l & 0xff));
                            data[i++] = (cs += ((l >> 8) & 0xff));
                            data[i++] = (cs += 0xf0);
                            data[i++] = (cs += disableNMEA[mDisCounter]);
                            data[i++] = (cs += 0); // disable
                            data[i++] = cs.ck_a;
                            data[i++] = cs.ck_b;
                            return i;
                        });
                        break;
                    case State::SetMessageInterval3N:
                        break;
                    case State::GetVersion:
                        IO::outl<debug>("# GPS GV");
                        mEvent = Event::None;
                        uart::fillSendBuffer([](auto& data){
                            Protocols::CheckSum cs;
                            uint8_t i = 0;
                            data[i++] = 0xb5;
                            data[i++] = 0x62;
                            data[i++] = (cs += 0x0a);
                            data[i++] = (cs += 0x04);
                            const uint16_t l = 0;
                            data[i++] = (cs += (l & 0xff));
                            data[i++] = (cs += ((l >> 8) & 0xff));
                            data[i++] = cs.ck_a;
                            data[i++] = cs.ck_b;
                            return i;
                        });
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
                static inline void onTransferComplete(const auto f) {
                    if (mActive) {
                        const auto f2 = [&](){
                            f();
                            event(Event::TransferComplete);
                            return false; // do not enable receiver
                        };
                        uart::Isr::onTransferComplete(f2);
                    }
                }
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
                // mPackagesCount = 0;
                return p;
            }
            // private:
            static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t) {
                if ((data[0] == '$') && (data[1] == 'G')) { // NMEA
                    ++mVCount;
                    return true;
                }
                else if ((data[0] == 0xb5) && (data[1] == 0x62)) { // UBLOX binary
                    ++mVCount;
                    return true;
                }
                else {
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
            static inline uint16_t mVCount = 0;
            static inline volatile etl::Event<Event> mEvent;
            static inline volatile State mState = State::Init;
            static inline External::Tick<systemTimer> mStateTick;
        };
    }
}
