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

#include "atomic.h"

#include "byte.h"
#include "etl/algorithm.h"
#include "etl/ranged.h"
#include "units.h"
#include "tick.h"
#include "dsp.h"
#include "vesc_types.h"

#include "debug_pin.h"

namespace RC::VESC {
    using namespace etl::literals;
    using namespace Units::literals;
    using namespace std::literals::chrono_literals;

    namespace Master {
        namespace V4 {
            template<uint8_t N, typename Config, typename MCU = DefaultMcu>
            struct Serial {
                using clock = Config::clock;
                using systemTimer = Config::systemTimer;
                using debug = Config::debug;
                using dmaChRW = Config::dmaChRW;
                using pin = Config::pin;
                using tp = Config::tp;

                struct UartConfig {
                    using Clock = clock;
                    using ValueType = uint8_t;
                    static inline constexpr size_t size = 256; // send size, buffer size
                    static inline constexpr size_t minSize = 11; // receive size
                    using DmaChannelWrite = dmaChRW;
                    using DmaChannelRead = dmaChRW;
                    static inline constexpr bool useDmaTCIsr = false;
                    static inline constexpr bool useIdleIsr = false;
                    static inline constexpr bool useRxToIsr = false;
                    static inline constexpr uint16_t rxToCount = 0;
                    using Adapter = void;
                    using Debug = struct {
                        using tp = void;
                    };
                };

                using uart = Mcu::Stm::V2::Uart<N, UartConfig, MCU>;

                static inline constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;

                static inline void init() {
                    IO::outl<debug>("# VEsc init");
                    __disable_irq();
                    mState = State::Init;
                    mEvent = Event::None;
                    mActive = true;
                    uart::init();
                    uart::template rxEnable<false>();
                    uart::baud(460'800);
                    uart::template halfDuplex<true>();
                    uart::template enableTCIsr<true>();
                    __enable_irq();
                    pin::afunction(af);
                    pin::template pullup<true>();
                }
                static inline void reset() {
                    IO::outl<debug>("# VEsc reset");
                    __disable_irq();
                    dmaChRW::enable(false);
                    uart::reset();
                    mActive = false;
                    __enable_irq();
                    pin::analog();
                }

                enum class State : uint8_t {Init, Run, GetVersion, GetValues, SendThrot};
                enum class Event : uint8_t {None, OK, Error, ReceiveComplete, SendComplete, SendThrot};

                static inline void event(const Event e) {
                    mEvent = e;
                }
                static inline void periodic() {
                    switch(mState) {
                    case State::Init:
                        break;
                    default:
                        if (mEvent.is(Event::ReceiveComplete)) {
                            readReply();
                        }
                        break;
                    }
                }
                static inline constexpr External::Tick<systemTimer> initTicks{2000ms};

                static inline void ratePeriodic() {
                    const auto oldState = mState;
                    ++mStateTick;
                    switch(mState) {
                    case State::Init:
                        mStateTick.on(initTicks, []{
                            mState = State::GetVersion;
                        });
                        break;
                    case State::GetVersion:
                        if (mEvent.is(Event::OK)) {
                            mState = State::Run;
                        }
                        break;
                    case State::Run:
                        if (mEvent.is(Event::SendThrot)) {
                            mState = State::SendThrot;
                        }
                        break;
                    case State::GetValues:
                        if (mEvent.is(Event::OK)) {
                            mState = State::Run;
                        }
                        break;
                    case State::SendThrot:
                        if (mEvent.is(Event::OK)) {
                            mState = State::Run;
                        }
                        break;
                    }
                    if (oldState != mState) {
                        mStateTick.reset();
                        switch(mState) {
                        case State::Init:
                            IO::outl<debug>("# Vesc init");
                            break;
                        case State::Run:
                            IO::outl<debug>("# Vesc run");
                            break;
                        case State::GetVersion:
                            IO::outl<debug>("# Vesc ver");
                            getVersion();
                            break;
                        case State::GetValues:
                            IO::outl<debug>("# Vesc val");
                            getValues();
                            break;
                        case State::SendThrot:
                            IO::outl<debug>("# Vesc throt");
                            sendThrottle();
                            break;
                        }
                    }
                }
                static inline void sendThrottle() {
                    char* const data = (char*)uart::outputBuffer();
                    CRC16 cs;
                    uint8_t n = 0;
                    data[n++] = 0x02;
                    data[n++] = 0x05;
                    cs += data[n++] = (uint8_t)CommPacketId::COMM_SET_DUTY;
                    data[n++] = mThrottle >> 24;
                    data[n++] = mThrottle >> 16;
                    data[n++] = mThrottle >> 8;
                    data[n++] = mThrottle;
                    data[n++] = cs >> 8;
                    data[n++] = cs;
                    data[n++] = 0x03;
                    send(n);
                }
                static inline void getValues() {
                    char* const data = (char*)uart::outputBuffer();
                    CRC16 cs;
                    uint8_t n = 0;
                    data[n++] = 0x02;
                    data[n++] = 0x01;
                    cs += data[n++] = (uint8_t)CommPacketId::COMM_GET_VALUES;
                    data[n++] = cs >> 8;
                    data[n++] = cs;
                    data[n++] = 0x03;
                    send(n);
                }
                static inline void getVersion() {
                    char* const data = (char*)uart::outputBuffer();
                    CRC16 cs;
                    uint8_t n = 0;
                    data[n++] = 0x02;
                    data[n++] = 0x01;
                    cs += data[n++] = (uint8_t)CommPacketId::COMM_FW_VERSION;
                    data[n++] = cs >> 8;
                    data[n++] = cs;
                    data[n++] = 0x03;
                    send(n);
                }

                static inline void rxEnable() {
                    if (mActive) {
                        uart::dmaReenable([]{
                            dmaChRW::clearTransferCompleteIF();
                            uart::dmaSetupRead2(UartConfig::size);
                        });
                        uart::template rxEnable<true>();
                    }
                }
                static inline void onIdleWithDma(const auto f) {
                    if (mActive) {
                        uart::onIdleWithDma(f);
                    }
                }
                static inline void onTransferComplete(const auto f) {
                    if (mActive) {
                        uart::onTransferComplete(f);
                    }
                }
                static inline void set(const uint16_t sbus) {
                    if (!mActive) return;
                    if (mState == State::Run) {
                        mThrottle = sbus2throt(sbus);
                        event(Event::SendThrot);
                    }
                }
                static inline void update() {
                }
                static inline uint16_t current() {
                    return mCurrent;
                }
                static inline uint16_t rpm() {
                    return mRpm;
                }

                private:
                static inline int32_t sbus2throt(const uint16_t sbus) {
                    int v = (sbus - 992) * 100'000 / 820;
                    return std::clamp(v, -100'000, 100'000);
                }

                static inline void send(const uint8_t n) {
                    uart::template rxEnable<false>();
                    uart::startSend(n);
                }

                static inline void readReply() {
                    if constexpr(!std::is_same_v<tp, void>) {
                        tp::set();
                    }
                    const char* const data = (char*)uart::readBuffer();
                    const uint16_t nread = uart::readCount();

                    if (data[0] != 0x02) {
                        event(Event::Error);
                        return;
                    }
                    const uint8_t length = data[1];
                    CRC16 cs;
                    cs += data[2];
                    const CommPacketId type = CommPacketId(data[2]);
                    for(uint8_t i = 0; i < (length - 1); ++i) {
                        cs += data[i + 3];
                    }
                    uint16_t vcs = (data[length + 3] << 8) | data[length + 4];

                    if (vcs != cs) {
                        event(Event::Error);
                        return;
                    }

                    uint16_t k = 0;
                    if (type == CommPacketId::COMM_FW_VERSION) {
                        mVersionMajor = (uint8_t)data[k++];
                        mVersionMinor = (uint8_t)data[k++];
                        for(uint16_t i = 0; i < mName.size(); ++i) {
                            mName[i] = (char)data[k++];
                            if (mName[i] == '\0')
                                break;
                        }
                        k += 12; // UUID
                        k++; // pairing
                        mFWTestVersionNumber = (uint8_t)data[k++];
                        mHWType = (uint8_t)data[k++]; // enum?
                    }
                    else if (type == CommPacketId::COMM_GET_VALUES) {
                        mTemperature = ((int32_t)data[0]) << 8;
                        mTemperature |= ((int32_t)data[1]);

                        mTemperatureMotor = ((int32_t)data[2]) << 8;
                        mTemperatureMotor |= ((int32_t)data[3]);

                        mCurrent = ((int32_t)data[4]) << 24;
                        mCurrent |= ((int32_t)data[5]) << 16;
                        mCurrent |= ((int32_t)data[6]) << 8;
                        mCurrent |= ((int32_t)data[7]);

                        if (mCurrent < 0) {
                            mCurrent = -mCurrent;
                        }

                        mCurrentIn = ((int32_t)data[8]) << 24;
                        mCurrentIn |= ((int32_t)data[9]) << 16;
                        mCurrentIn |= ((int32_t)data[10]) << 8;
                        mCurrentIn |= ((int32_t)data[11]);

                        if (mCurrentIn < 0) {
                            mCurrentIn = -mCurrentIn;
                        }

                        mRpm = ((int32_t)data[22]) << 24;
                        mRpm |= ((int32_t)data[23]) << 16;
                        mRpm |= ((int32_t)data[24]) << 8;
                        mRpm |= ((int32_t)data[25]);

                        if (mRpm < 0) {
                            mRpm = -mRpm;
                        }

                        mVoltage = ((int32_t)data[26]) << 8;
                        mVoltage |= ((int32_t)data[27]);

                        mConsumption = ((int32_t)data[28]) << 24;
                        mConsumption |= ((int32_t)data[29]) << 16;
                        mConsumption |= ((int32_t)data[30]) << 8;
                        mConsumption |= ((int32_t)data[31]);

                        mFault = (uint8_t)data[52];
                    }
                    event(Event::OK);
                }

                static inline int32_t mThrottle{ 0 };
                static inline uint32_t mTemperature;
                static inline uint32_t mTemperatureMotor;
                static inline uint32_t mVoltage;
                static inline int32_t mCurrent;
                static inline int32_t mCurrentIn;
                static inline uint32_t mConsumption;
                static inline int32_t mRpm;
                static inline uint8_t mVersionMajor;
                static inline uint8_t mVersionMinor;
                static inline uint8_t mFWTestVersionNumber;
                static inline uint8_t mHWType;
                static inline uint8_t mFault;
                static inline std::array<char, 256> mName;
                static inline volatile etl::Event<Event> mEvent;
                static inline volatile State mState = State::Init;
                static inline External::Tick<systemTimer> mStateTick;
                static inline volatile bool mActive = false;
            };
        }

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

