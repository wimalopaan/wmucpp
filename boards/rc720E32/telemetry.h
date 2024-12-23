#pragma once

#include "rc/crsf.h"

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
            mMessage[mCounter++] = std::byte{0xc8};
            mCounter++;
            crc += mMessage[mCounter++] = RC::Protokoll::Crsf::Type::ArduPilot;
            // the next two bytes should contain ext. destination and source, but
            // nobody (yaapu, betaflight, ...) does this.
            crc += mMessage[mCounter++] = RC::Protokoll::Crsf::Address::Handset;
            crc += mMessage[mCounter++] = std::byte(storage::eeprom.address);

            crc += mMessage[mCounter++] = std::byte(mDataID >> 8);
            crc += mMessage[mCounter++] = std::byte(mDataID & 0xff);
            for(uint16_t i = 0; i < mValues.size(); ++i) {
                crc += mMessage[mCounter++] = std::byte(mValues[i] >> 8);
                crc += mMessage[mCounter++] = std::byte(mValues[i] & 0xff);
            }
            crc += mMessage[mCounter++] = std::byte(mTurns[0]);
            crc += mMessage[mCounter++] = std::byte(mTurns[1]);
            crc += mMessage[mCounter++] = std::byte(mFlags);
            mMessage[1] = std::byte(mCounter - 1);
            mMessage[mCounter++] = crc;
            buffer::enqueue(std::span<std::byte>(std::begin(mMessage), mCounter));
        }
        else {
            mFrameCounter = 0;
            mMessage[mCounter++] = std::byte{0xc8};
            mCounter++;
            crc += mMessage[mCounter++] = RC::Protokoll::Crsf::Type::ArduPilot;
            // the next two bytes should contain ext. destination and source, but
            // nobody (yaapu, betaflight, ...) does this.
            crc += mMessage[mCounter++] = RC::Protokoll::Crsf::Address::Handset;
            crc += mMessage[mCounter++] = std::byte(storage::eeprom.address);

            crc += mMessage[mCounter++] = std::byte(mInfoID >> 8);
            crc += mMessage[mCounter++] = std::byte(mInfoID & 0xff);

            crc += mMessage[mCounter++] = std::byte{servos::fwVersion(0).first};
            crc += mMessage[mCounter++] = std::byte{servos::fwVersion(0).second};
            crc += mMessage[mCounter++] = std::byte{servos::hwVersion(0).first};
            crc += mMessage[mCounter++] = std::byte{servos::hwVersion(0).second};
            crc += mMessage[mCounter++] = std::byte{servos::fwVersion(1).first};
            crc += mMessage[mCounter++] = std::byte{servos::fwVersion(1).second};
            crc += mMessage[mCounter++] = std::byte{servos::hwVersion(1).first};
            crc += mMessage[mCounter++] = std::byte{servos::hwVersion(1).second};

            crc += mMessage[mCounter++] = std::byte{escs::fwVersion(0).first};
            crc += mMessage[mCounter++] = std::byte{escs::fwVersion(0).second};
            crc += mMessage[mCounter++] = std::byte{escs::hwVersion(0).first};
            crc += mMessage[mCounter++] = std::byte{escs::hwVersion(0).second};
            crc += mMessage[mCounter++] = std::byte{escs::fwVersion(1).first};
            crc += mMessage[mCounter++] = std::byte{escs::fwVersion(1).second};
            crc += mMessage[mCounter++] = std::byte{escs::hwVersion(1).first};
            crc += mMessage[mCounter++] = std::byte{escs::hwVersion(1).second};

            mMessage[1] = std::byte(mCounter - 1);
            mMessage[mCounter++] = crc;
            buffer::enqueue(std::span<std::byte>(std::begin(mMessage), mCounter));
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
    static inline std::array<std::byte, 64> mMessage;
    static inline uint16_t mDataID = 6000;
    static inline std::array<uint16_t, 10> mValues{};
    static inline std::array<int8_t, 2> mTurns{};
    static inline uint8_t mFlags = 0;
    static inline uint8_t mCounter = 0;
};

