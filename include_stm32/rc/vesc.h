#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <numbers>
#include <chrono>

#include "byte.h"
#include "etl/algorithm.h"
#include "etl/ranged.h"
#include "units.h"
#include "tick.h"
#include "dsp.h"

#include "vesc_types.h"

namespace RC::VESC {
    using namespace etl::literals;
    using namespace Units::literals;
    using namespace std::literals::chrono_literals;

    struct CheckSum {
        void operator+=(const std::byte b) {
            mValue = std::byte(tbl[uint8_t(mValue ^ b)]);
        }
        void reset() {
            mValue = std::byte{ 0 };
        }
        operator std::byte() const {
            return mValue;
        }
    private:
        std::byte mValue{};
        inline static constexpr uint8_t tbl[] = {
            0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
            0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
            0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
            0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
            0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
            0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
            0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
            0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
            0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
            0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
            0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
            0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
            0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
            0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
            0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
            0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3,
        };
    };

    namespace Master {
        namespace V3 {
            template<uint8_t N, typename Config, typename Timer, typename MCU = DefaultMcu>
            struct ProtocolAdapter {

                using CB = Config::callback;

                enum class State : uint8_t { Idle, WaitForResponse, Length, Type, Data, CrcH, CrcL, End };

                static inline void startWait(State s) {
                    mState = s;
                }
                static inline void stopWait() {
                    mState = State::Idle;
                }

                static inline void process(const std::byte b) {
                    static CRC16 csum;
                    ++mBytes;
                    switch (mState) {
                    case State::Idle:
                        break;
                    case State::WaitForResponse:
                        if (b == 0x02_B) {
                            mState = State::Length;
                        }
                        else {
                            mState = State::Idle;
                        }
                        break;
                    case State::Length:
                        mLength = (uint8_t)b;
                        csum.reset();
                        mState = State::Type;
                        break;
                    case State::Type:
                        --mLength;
                        csum += b;
                        mType = CommPacketId((uint8_t)b);
                        mState = State::Data;
                        mDataCounter = 0;
                        break;
                    case State::Data:
                        csum += b;
                        mData[mDataCounter++] = b;
                        if (mDataCounter == mLength) {
                            mState = State::CrcH;
                        }
                        break;
                    case State::CrcH:
                        mCRC = ((uint16_t)b) << 8;
                        mState = State::CrcL;
                        break;
                    case State::CrcL:
                        mCRC |= (uint8_t)b;
                        if ((uint16_t)csum == mCRC) {
                            decode();
                            CB::update();
                            ++mPackages;
                        }
                        mState = State::End;
                        break;
                    case State::End:
                        mState = State::Idle;
                        break;
                    }
                }

                static inline uint16_t mPackages{};
                static inline uint16_t mBytes{};
            // private:
                static inline void decode() {
                    if (mType == CommPacketId::COMM_FW_VERSION) {
                        uint16_t k = 0;
                        mVersionMajor = (uint8_t)mData[k++];
                        mVersionMinor = (uint8_t)mData[k++];
                        for(uint16_t i = 0; i < mName.size(); ++i) {
                            mName[i] = (char)mData[k++];
                            if (mName[i] == '\0')
                                break;
                        }
                        k += 12; // UUID
                        k++; // pairing
                        mFWTestVersionNumber = (uint8_t)mData[k++];
                        mHWType = (uint8_t)mData[k++]; // enum?

                        CB::setFWInfo(mVersionMajor, mVersionMinor, &mName[0]);
                    }
                    else if (mType == CommPacketId::COMM_GET_VALUES) {
                        mTemperature = ((int32_t)mData[0]) << 8;
                        mTemperature |= ((int32_t)mData[1]);

                        mTemperatureMotor = ((int32_t)mData[2]) << 8;
                        mTemperatureMotor |= ((int32_t)mData[3]);

                        mCurrent = ((int32_t)mData[4]) << 24;
                        mCurrent |= ((int32_t)mData[5]) << 16;
                        mCurrent |= ((int32_t)mData[6]) << 8;
                        mCurrent |= ((int32_t)mData[7]);

                        if (mCurrent < 0) {
                            mCurrent = -mCurrent;
                        }

                        mCurrentIn = ((int32_t)mData[8]) << 24;
                        mCurrentIn |= ((int32_t)mData[9]) << 16;
                        mCurrentIn |= ((int32_t)mData[10]) << 8;
                        mCurrentIn |= ((int32_t)mData[11]);

                        if (mCurrentIn < 0) {
                            mCurrentIn = -mCurrentIn;
                        }

                        mRPM = ((int32_t)mData[22]) << 24;
                        mRPM |= ((int32_t)mData[23]) << 16;
                        mRPM |= ((int32_t)mData[24]) << 8;
                        mRPM |= ((int32_t)mData[25]);

                        if (mRPM < 0) {
                            mRPM = -mRPM;
                        }

                        mVoltage = ((int32_t)mData[26]) << 8;
                        mVoltage |= ((int32_t)mData[27]);

                        mConsumption = ((int32_t)mData[28]) << 24;
                        mConsumption |= ((int32_t)mData[29]) << 16;
                        mConsumption |= ((int32_t)mData[30]) << 8;
                        mConsumption |= ((int32_t)mData[31]);

                        mFault = (uint8_t)mData[52];
                    }

                }
                static inline std::array<char, 256> mName;
                static inline std::array<std::byte, 256> mData;
                static inline uint8_t mLength{};
                static inline CommPacketId mType{};
                static inline uint16_t mCRC{};
                static inline uint8_t mDataCounter;
                static inline uint32_t mTemperature;
                static inline uint32_t mTemperatureMotor;
                static inline uint32_t mVoltage;
                static inline int32_t mCurrent;
                static inline int32_t mCurrentIn;
                static inline uint32_t mConsumption;
                static inline int32_t mRPM;
                static inline uint8_t mVersionMajor;
                static inline uint8_t mVersionMinor;
                static inline uint8_t mFWTestVersionNumber;
                static inline uint8_t mHWType;
                static inline uint8_t mFault;
                static inline State mState{ State::Idle };
            };

            template<typename UART, typename Timer, typename MCU = DefaultMcu>
            struct Fsm {
                using uart = UART;
                using pa_t = uart::pa_t;

                enum class State : uint8_t { Version, GetValues, SendThrottle, WaitComplete, Wait };
                static inline External::Tick<Timer> waitTicks{ 20ms };

                static inline void periodic() {
                    switch (mState) {
                    case State::WaitComplete:
                        if (uart::isIdle() && uart::isTxQueueEmpty()) {
                            pa_t::startWait(pa_t::State::WaitForResponse);
                            uart::clear();
                            uart::template rxEnable<true>();
                            mState = State::Wait;
                        }
                        break;
                    default:
                        break;
                    }
                }

                static inline void ratePeriodic() {
                    ++mStateTick;
                    switch (mState) {
                    case State::Version:
                    {
                        CRC16 cs;
                        uart::template rxEnable<false>();
                        uart::put(0x02_B);
                        uart::put(0x01_B);
                        cs += uart::put(std::byte(CommPacketId::COMM_FW_VERSION));
                        uart::put(std::byte(((uint16_t)cs) >> 8));
                        uart::put(std::byte(((uint16_t)cs)));
                        uart::put(0x03_B);
                        mNextState = State::SendThrottle;
                        mState = State::WaitComplete;
                    }
                        break;
                    case State::SendThrottle:
                    {
                        CRC16 cs;
                        uart::template rxEnable<false>();
                        uart::put(0x02_B);
                        uart::put(0x05_B);
                        cs += uart::put(std::byte(CommPacketId::COMM_SET_DUTY));
                        cs += uart::put(std::byte(mThrottle >> 24));
                        cs += uart::put(std::byte(mThrottle >> 16));
                        cs += uart::put(std::byte(mThrottle >> 8));
                        cs += uart::put(std::byte(mThrottle));
                        uart::put(std::byte(((uint16_t)cs) >> 8));
                        uart::put(std::byte(((uint16_t)cs)));
                        uart::put(0x03_B);
                        mNextState = State::GetValues;
                        mState = State::WaitComplete;
                    }
                    break;
                    case State::GetValues:
                    {
                        CRC16 cs;
                        uart::template rxEnable<false>();
                        uart::put(0x02_B);
                        uart::put(0x01_B);
                        cs += uart::put(std::byte(CommPacketId::COMM_GET_VALUES));
                        uart::put(std::byte(((uint16_t)cs) >> 8));
                        uart::put(std::byte(((uint16_t)cs)));
                        uart::put(0x03_B);
                        if (pa_t::mVersionMajor > 0) {
                            mNextState = State::SendThrottle;
                        }
                        else {
                            mNextState = State::Version;
                        }
                        mState = State::WaitComplete;
                    }
                    break;
                    case State::Wait:
                        mStateTick.on(waitTicks, [] {
                            pa_t::stopWait();
                            mState = mNextState;
                            });
                        break;
                    default:
                        break;
                    }
                }
                // template<auto L, auto H>
                // static inline void throttle(const etl::ranged<L, H> v) {
                // }
                static inline void throttle(const uint16_t sbus) {
                    static constexpr int dead = 10;
                    float diff = sbus - 992.0f;
                    float v = 0;
                    if (fabs(diff) < dead) {
                        v = 0;
                    }
                    else {
                        if (diff > 0) {
                            v = (diff - dead) * 840.0f / (840.0f - dead);
                        }
                        else {
                            v = (diff + dead) * 840.0f / (840.0f - dead);
                        }
                    }
                    expMean.process(v);
                    mThrottle = std::clamp(100'000.0f * (expMean.value() / 840.0f), -100'000.0f, 100'000.0f);
                }
                static inline void inertia(const float f) {
                    expMean.factor(f);
                }
            private:
                static inline Dsp::ExpMean<void> expMean{0.001};
                static inline int32_t mThrottle{ 0 };
                static inline uint8_t mBrake{ 0 };
                static inline uint8_t mLed{ 0 };

                static inline External::Tick<Timer> mStateTick;
                static inline State mState{ State::SendThrottle };
                static inline State mNextState{ mState };
            };
        }
        namespace V2 {
            template<uint8_t N, typename Config, typename Timer, typename MCU = DefaultMcu>
            struct ProtocolAdapter {

                using CB = Config::callback;

                enum class State : uint8_t { Idle, WaitForResponse, Length, Type, Data, CrcH, CrcL, End };

                static inline void startWait(State s) {
                    mState = s;
                }
                static inline void stopWait() {
                    mState = State::Idle;
                }

                static inline void process(const std::byte b) {
                    static CRC16 csum;
                    ++mBytes;
                    switch (mState) {
                    case State::Idle:
                        break;
                    case State::WaitForResponse:
                        if (b == 0x02_B) {
                            mState = State::Length;
                        }
                        else {
                            mState = State::Idle;
                        }
                        break;
                    case State::Length:
                        mLength = (uint8_t)b;
                        csum.reset();
                        mState = State::Type;
                        break;
                    case State::Type:
                        --mLength;
                        csum += b;
                        mType = CommPacketId((uint8_t)b);
                        mState = State::Data;
                        mDataCounter = 0;
                        break;
                    case State::Data:
                        csum += b;
                        mData[mDataCounter++] = b;
                        if (mDataCounter == mLength) {
                            mState = State::CrcH;
                        }
                        break;
                    case State::CrcH:
                        mCRC = ((uint16_t)b) << 8;
                        mState = State::CrcL;
                        break;
                    case State::CrcL:
                        mCRC |= (uint8_t)b;
                        if ((uint16_t)csum == mCRC) {
                            decode();
                            CB::update();
                            ++mPackages;
                        }
                        mState = State::End;
                        break;
                    case State::End:
                        mState = State::Idle;
                        break;
                    }
                }

                static inline uint16_t mPackages{};
                static inline uint16_t mBytes{};
            // private:
                static inline void decode() {
                    if (mType == CommPacketId::COMM_FW_VERSION) {
                        uint16_t k = 0;
                        mVersionMajor = (uint8_t)mData[k++];
                        mVersionMinor = (uint8_t)mData[k++];
                        for(uint16_t i = 0; i < mName.size(); ++i) {
                            mName[i] = (char)mData[k++];
                            if (mName[i] == '\0')
                                break;
                        }
                        k += 12; // UUID
                        k++; // pairing
                        mFWTestVersionNumber = (uint8_t)mData[k++];
                        mHWType = (uint8_t)mData[k++]; // enum?

                        CB::setFWInfo(mVersionMajor, mVersionMinor, &mName[0]);
                    }
                    else if (mType == CommPacketId::COMM_GET_VALUES) {
                        mTemperature = ((int32_t)mData[0]) << 8;
                        mTemperature |= ((int32_t)mData[1]);

                        mTemperatureMotor = ((int32_t)mData[2]) << 8;
                        mTemperatureMotor |= ((int32_t)mData[3]);

                        mCurrent = ((int32_t)mData[4]) << 24;
                        mCurrent |= ((int32_t)mData[5]) << 16;
                        mCurrent |= ((int32_t)mData[6]) << 8;
                        mCurrent |= ((int32_t)mData[7]);

                        if (mCurrent < 0) {
                            mCurrent = -mCurrent;
                        }

                        mCurrentIn = ((int32_t)mData[8]) << 24;
                        mCurrentIn |= ((int32_t)mData[9]) << 16;
                        mCurrentIn |= ((int32_t)mData[10]) << 8;
                        mCurrentIn |= ((int32_t)mData[11]);

                        if (mCurrentIn < 0) {
                            mCurrentIn = -mCurrentIn;
                        }

                        mRPM = ((int32_t)mData[22]) << 24;
                        mRPM |= ((int32_t)mData[23]) << 16;
                        mRPM |= ((int32_t)mData[24]) << 8;
                        mRPM |= ((int32_t)mData[25]);

                        if (mRPM < 0) {
                            mRPM = -mRPM;
                        }

                        mVoltage = ((int32_t)mData[26]) << 8;
                        mVoltage |= ((int32_t)mData[27]);

                        mConsumption = ((int32_t)mData[28]) << 24;
                        mConsumption |= ((int32_t)mData[29]) << 16;
                        mConsumption |= ((int32_t)mData[30]) << 8;
                        mConsumption |= ((int32_t)mData[31]);

                        mFault = (uint8_t)mData[52];
                    }

                }
                static inline std::array<char, 256> mName;
                static inline std::array<std::byte, 256> mData;
                static inline uint8_t mLength{};
                static inline CommPacketId mType{};
                static inline uint16_t mCRC{};
                static inline uint8_t mDataCounter;
                static inline uint32_t mTemperature;
                static inline uint32_t mTemperatureMotor;
                static inline uint32_t mVoltage;
                static inline int32_t mCurrent;
                static inline int32_t mCurrentIn;
                static inline uint32_t mConsumption;
                static inline int32_t mRPM;
                static inline uint8_t mVersionMajor;
                static inline uint8_t mVersionMinor;
                static inline uint8_t mFWTestVersionNumber;
                static inline uint8_t mHWType;
                static inline uint8_t mFault;
                static inline State mState{ State::Idle };
            };

            template<typename UART, typename Timer, typename MCU = DefaultMcu>
            struct Fsm {
                using uart = UART;
                using pa_t = uart::pa_t;

                enum class State : uint8_t { Version, GetValues, SendThrottle, WaitComplete, Wait };
                static inline External::Tick<Timer> waitTicks{ 20ms };

                static inline void periodic() {
                    switch (mState) {
                    case State::WaitComplete:
                        if (uart::isIdle() && uart::isTxQueueEmpty()) {
                            pa_t::startWait(pa_t::State::WaitForResponse);
                            uart::clear();
                            uart::template rxEnable<true>();
                            mState = State::Wait;
                        }
                        break;
                    default:
                        break;
                    }
                }

                static inline void ratePeriodic() {
                    ++mStateTick;
                    switch (mState) {
                    case State::Version:
                    {
                        CRC16 cs;
                        uart::template rxEnable<false>();
                        uart::put(0x02_B);
                        uart::put(0x01_B);
                        cs += uart::put(std::byte(CommPacketId::COMM_FW_VERSION));
                        uart::put(std::byte(((uint16_t)cs) >> 8));
                        uart::put(std::byte(((uint16_t)cs)));
                        uart::put(0x03_B);
                        mNextState = State::SendThrottle;
                        mState = State::WaitComplete;
                    }
                        break;
                    case State::SendThrottle:
                    {
                        CRC16 cs;
                        uart::template rxEnable<false>();
                        uart::put(0x02_B);
                        uart::put(0x05_B);
                        cs += uart::put(std::byte(CommPacketId::COMM_SET_DUTY));
                        cs += uart::put(std::byte(mThrottle >> 24));
                        cs += uart::put(std::byte(mThrottle >> 16));
                        cs += uart::put(std::byte(mThrottle >> 8));
                        cs += uart::put(std::byte(mThrottle));
                        uart::put(std::byte(((uint16_t)cs) >> 8));
                        uart::put(std::byte(((uint16_t)cs)));
                        uart::put(0x03_B);
                        mNextState = State::GetValues;
                        mState = State::WaitComplete;
                    }
                    break;
                    case State::GetValues:
                    {
                        CRC16 cs;
                        uart::template rxEnable<false>();
                        uart::put(0x02_B);
                        uart::put(0x01_B);
                        cs += uart::put(std::byte(CommPacketId::COMM_GET_VALUES));
                        uart::put(std::byte(((uint16_t)cs) >> 8));
                        uart::put(std::byte(((uint16_t)cs)));
                        uart::put(0x03_B);
                        if (pa_t::mVersionMajor > 0) {
                            mNextState = State::SendThrottle;
                        }
                        else {
                            mNextState = State::Version;
                        }
                        mState = State::WaitComplete;
                    }
                    break;
                    case State::Wait:
                        mStateTick.on(waitTicks, [] {
                            pa_t::stopWait();
                            mState = mNextState;
                            });
                        break;
                    default:
                        break;
                    }
                }
                // template<auto L, auto H>
                // static inline void throttle(const etl::ranged<L, H> v) {
                // }
                static inline void throttle(const uint16_t sbus) {
                    static constexpr int dead = 10;
                    float diff = sbus - 992.0f;
                    float v = 0;
                    if (fabs(diff) < dead) {
                        v = 0;
                    }
                    else {
                        if (diff > 0) {
                            v = (diff - dead) * 840.0f / (840.0f - dead);
                        }
                        else {
                            v = (diff + dead) * 840.0f / (840.0f - dead);
                        }
                    }
                    expMean.process(v);
                    mThrottle = std::clamp(100'000.0f * (expMean.value() / 840.0f), -100'000.0f, 100'000.0f);
                }
                static inline void inertia(const float f) {
                    expMean.factor(f);
                }
            private:
                static inline Dsp::ExpMean<void> expMean{0.001};
                static inline int32_t mThrottle{ 0 };
                static inline uint8_t mBrake{ 0 };
                static inline uint8_t mLed{ 0 };

                static inline External::Tick<Timer> mStateTick;
                static inline State mState{ State::SendThrottle };
                static inline State mNextState{ mState };
            };
        }

        namespace V1 {
            template<uint8_t N, typename CB, typename Timer, typename MCU = DefaultMcu>
            struct ProtocolAdapter {

                enum class State : uint8_t { Idle, WaitForResponse, Length, Type, Data, CrcH, CrcL, End };

                static inline void startWait(State s) {
                    mState = s;
                }
                static inline void stopWait() {
                    mState = State::Idle;
                }

                static inline void process(const std::byte b) {
                    static CRC16 csum;
                    ++mBytes;
                    switch (mState) {
                    case State::Idle:
                        break;
                    case State::WaitForResponse:
                        if (b == 0x02_B) {
                            mState = State::Length;
                        }
                        else {
                            mState = State::Idle;
                        }
                        break;
                    case State::Length:
                        mLength = (uint8_t)b;
                        csum.reset();
                        mState = State::Type;
                        break;
                    case State::Type:
                    --mLength;
                        csum += b;
                        mType = CommPacketId((uint8_t)b);
                        mState = State::Data;
                        mDataCounter = 0;
                        break;
                    case State::Data:
                        csum += b;
                        mData[mDataCounter++] = b;
                        if (mDataCounter == mLength) {
                            mState = State::CrcH;
                        }
                        break;
                    case State::CrcH:
                        mCRC = ((uint16_t)b) << 8;
                        mState = State::CrcL;
                        break;
                    case State::CrcL:
                        mCRC |= (uint8_t)b;
                        if ((uint16_t)csum == mCRC) {
                            decode();
                            CB::update();
                            ++mPackages;
                        }
                        mState = State::End;
                        break;
                    case State::End:
                        mState = State::Idle;
                        break;
                    }
                }

                static inline uint16_t mPackages{};
                static inline uint16_t mBytes{};
            // private:
                static inline void decode() {
                    if (mType == CommPacketId::COMM_GET_VALUES) {
                        mTemperature = ((int32_t)mData[0]) << 8;
                        mTemperature |= ((int32_t)mData[1]);

                        mCurrent = ((int32_t)mData[4]) << 24;
                        mCurrent |= ((int32_t)mData[5]) << 16;
                        mCurrent |= ((int32_t)mData[6]) << 8;
                        mCurrent |= ((int32_t)mData[7]);

                        if (mCurrent < 0) {
                            mCurrent = -mCurrent;
                        }

                        mRPM = ((int32_t)mData[22]) << 24;
                        mRPM |= ((int32_t)mData[23]) << 16;
                        mRPM |= ((int32_t)mData[24]) << 8;
                        mRPM |= ((int32_t)mData[25]);

                        if (mRPM < 0) {
                            mRPM = -mRPM;
                        }

                        mVoltage = ((int32_t)mData[26]) << 8;
                        mVoltage |= ((int32_t)mData[27]);
                    }

                }
                static inline std::array<std::byte, 256> mData;
                static inline uint8_t mLength{};
                static inline CommPacketId mType{};
                static inline uint16_t mCRC{};
                static inline uint8_t mDataCounter;
                static inline uint32_t mTemperature;
                static inline uint32_t mVoltage;
                static inline int32_t mCurrent;
                static inline uint32_t mConsumption;
                static inline int32_t mRPM;
                static inline State mState{ State::Idle };
            };

            template<typename UART, typename Timer, typename MCU = DefaultMcu>
            struct Fsm {
                using uart = UART;
                using pa_t = uart::pa_t;

                enum class State : uint8_t { Version, GetValues, SendThrottle, WaitComplete, Wait };
                static inline External::Tick<Timer> waitTicks{ 20ms };

                static inline void periodic() {
                    switch (mState) {
                    case State::WaitComplete:
                        if (uart::isIdle() && uart::isTxQueueEmpty()) {
                            pa_t::startWait(pa_t::State::WaitForResponse);
                           uart::clear();
                            uart::template rxEnable<true>();
                            mState = State::Wait;
                        }
                        break;
                    default:
                        break;
                    }
                }

                static inline void ratePeriodic() {
                    ++mStateTick;
                    switch (mState) {
                    case State::Version:
                    {
                        CRC16 cs;
                        uart::template rxEnable<false>();
                        uart::put(0x02_B);
                        uart::put(0x01_B);
                        cs += uart::put(std::byte(CommPacketId::COMM_FW_VERSION));
                        uart::put(std::byte(((uint16_t)cs) >> 8));
                        uart::put(std::byte(((uint16_t)cs)));
                        uart::put(0x03_B);
                        mNextState = State::SendThrottle;
                        mState = State::WaitComplete;
                    }
                        break;
                    case State::SendThrottle:
                    {
                        CRC16 cs;
                        uart::template rxEnable<false>();
                        uart::put(0x02_B);
                        uart::put(0x05_B);
                        cs += uart::put(std::byte(CommPacketId::COMM_SET_DUTY));
                        cs += uart::put(std::byte(mThrottle >> 24));
                        cs += uart::put(std::byte(mThrottle >> 16));
                        cs += uart::put(std::byte(mThrottle >> 8));
                        cs += uart::put(std::byte(mThrottle));
                        uart::put(std::byte(((uint16_t)cs) >> 8));
                        uart::put(std::byte(((uint16_t)cs)));
                        uart::put(0x03_B);
                        mNextState = State::GetValues;
                        mState = State::WaitComplete;
                    }
                    break;
                    case State::GetValues:
                    {
                        CRC16 cs;
                        uart::template rxEnable<false>();
                        uart::put(0x02_B);
                        uart::put(0x01_B);
                        cs += uart::put(std::byte(CommPacketId::COMM_GET_VALUES));
                        uart::put(std::byte(((uint16_t)cs) >> 8));
                        uart::put(std::byte(((uint16_t)cs)));
                        uart::put(0x03_B);
                        mNextState = State::SendThrottle;
                        mState = State::WaitComplete;
                    }
                    break;
                    case State::Wait:
                        mStateTick.on(waitTicks, [] {
                            pa_t::stopWait();
                            mState = mNextState;
                            });
                        break;
                    default:
                        break;
                    }
                }
                // template<auto L, auto H>
                // static inline void throttle(const etl::ranged<L, H> v) {
                // }
                static inline void throttle(const uint16_t sbus) {
                    static constexpr int dead = 10;
                    float diff = sbus - 992.0f;
                    float v = 0;
                    if (fabs(diff) < dead) {
                        v = 0;
                    }
                    else {
                        if (diff > 0) {
                            v = (diff - dead) * 840.0f / (840.0f - dead);
                        }
                        else {
                            v = (diff + dead) * 840.0f / (840.0f - dead);
                        }
                    }
                    expMean.process(v);
                    mThrottle = std::clamp(100'000.0f * (expMean.value() / 840.0f), -100'000.0f, 100'000.0f);
                }

            private:
                static inline Dsp::ExpMean<void> expMean{0.0005};
                static inline int32_t mThrottle{ 0 };
                static inline uint8_t mBrake{ 0 };
                static inline uint8_t mLed{ 0 };

                static inline External::Tick<Timer> mStateTick;
                static inline State mState{ State::SendThrottle };
                static inline State mNextState{ mState };
            };

        }
    }
}

