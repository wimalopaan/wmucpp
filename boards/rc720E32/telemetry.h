#pragma once

#include "rc/crsf.h"

template<typename Dev, typename Storage, typename Debug>
struct Telemetry {
    using debug = Debug;
    using uart = Dev;
    using storage = Storage;

    static inline void next() {
        if (uart::outputBuffer()) {
            mCounter = 0;
            CRC8 crc;
            addToBuffer(std::byte{0xc8});
            addToBuffer(std::byte{0});
            crc += addToBuffer(RC::Protokoll::Crsf::Type::ArduPilot);
            crc += addToBuffer(std::byte(mDataID >> 8));
            crc += addToBuffer(std::byte(mDataID & 0xff));
            crc += addToBuffer(std::byte(storage::eeprom.address));
            for(uint16_t i = 0; i < mValues.size(); ++i) {
                crc += addToBuffer(std::byte(mValues[i] >> 8));
                crc += addToBuffer(std::byte(mValues[i] & 0xff));
            }
            crc += addToBuffer(std::byte(mTurns[0]));
            crc += addToBuffer(std::byte(mTurns[1]));
            crc += addToBuffer(std::byte(mFlags));
            uart::outputBuffer()[1] = std::byte(mCounter - 1);
            uart::outputBuffer()[mCounter++] = crc;
            uart::startSend(mCounter);
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
    // static inline void next() {
    //     if (uart::outputBuffer()) {
    //         mCounter = 0;
    //         CRC8 crc;
    //         addToBuffer(std::byte{0xc8});
    //         addToBuffer(std::byte{0});
    //         crc += addToBuffer(RC::Protokoll::Crsf::Type::FlightMode);
    //         crc += addToBuffer(std::byte{'N'});
    //         crc += addToBuffer(std::byte{'N'});
    //         crc += addToBuffer(std::byte{'\0'});
    //         uart::outputBuffer()[1] = std::byte(mCounter - 1);
    //         uart::outputBuffer()[mCounter++] = crc;
    //         uart::startSend(mCounter);
    //     }
    // }

    private:
    static inline uint16_t mDataID = 6000;
    static inline std::array<uint16_t, 10> mValues{};
    static inline std::array<int8_t, 2> mTurns{};
    static inline uint8_t mFlags = 0;
    static inline uint8_t mCounter = 0;
    static inline std::byte addToBuffer(const std::byte b) {
        uart::outputBuffer()[mCounter++] = b;
        return b;
    }
};

