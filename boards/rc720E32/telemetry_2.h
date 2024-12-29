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

    static inline void next() {
        mCounter = 0;
        CRC8 crc;
        if (++mFrameCounter < infoRate) {
            buffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::ArduPilot, [&](auto& d){
                d.push_back(RC::Protokoll::Crsf::V4::Address::Handset);
                d.push_back((uint8_t)storage::eeprom.address);
                d.push_back(mDataID);
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
                d.push_back(mInfoID);

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
    static inline uint16_t mInfoID = 6001;
    // static inline std::array<uint8_t, RC::Protokoll::Crsf::V4::maxMessageSize> mMessage;
    static inline uint16_t mDataID = 6000;
    static inline std::array<uint16_t, 10> mValues{};
    static inline std::array<int8_t, 2> mTurns{};
    static inline uint8_t mFlags = 0;
    static inline uint8_t mCounter = 0;
};

