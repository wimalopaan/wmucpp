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
#include "usart_2.h"

namespace RC::VESC {
    using namespace std::literals::chrono_literals;

    namespace Master {
        namespace V5 {
            template<uint8_t N, typename Config, typename MCU = DefaultMcu>
            struct Serial {
                using clock = Config::clock;
                using systemTimer = Config::systemTimer;
                using debug = Config::debug;
                using dmaChComponent = Config::dmaChComponent;
                using pin = Config::pin;
                using tp = Config::tp;

                private:
                struct UartConfig;
                template<typename> struct ProtocolAdapter;
                using protocol_adapter = ProtocolAdapter<Serial>;

                public:
                using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;
                struct TelemetryValues {
                    static inline uint32_t mTemperature;
                    static inline uint32_t mTemperatureMotor;
                    static inline uint32_t mVoltage;
                    static inline int32_t mCurrent;
                    static inline int32_t mCurrentIn;
                    static inline uint32_t mConsumption;
                    static inline int32_t mRpm;
                    static inline uint8_t mFault;
                };
                struct DeviceInfo {
                    static inline uint8_t mVersionMajor;
                    static inline uint8_t mVersionMinor;
                    static inline std::array<char, 64> mName;
                    static inline uint8_t mFWTestVersionNumber;
                    static inline uint8_t mHWType;
                };

                static inline void init() {
                    static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;
                    IO::outl<debug>("# VEsc init");
                    Mcu::Arm::Atomic::access([]{
                        mState = State::Init;
                        mEvent = Event::None;
                        mActive = true;
                        uart::init();
                    });
                    pin::afunction(af);
                    pin::template pullup<true>();
                }
                static inline void reset() {
                    IO::outl<debug>("# VEsc reset");
                    Mcu::Arm::Atomic::access([]{
                        uart::reset();
                        mActive = false;
                    });
                    pin::analog();
                }

                enum class State : uint8_t {Init, Run, GetVersion, GetValues, SendThrot};
                enum class Event : uint8_t {None, OK, Error, SendThrot,
                                           ReceiveComplete};

                static inline constexpr External::Tick<systemTimer> retryTicks{1000ms};
                static inline constexpr External::Tick<systemTimer> initTicks{2000ms};
                static inline constexpr External::Tick<systemTimer> cycleTicks{20ms};

                static inline void event(const Event e) {
                    mEvent = e;
                }
                static inline void periodic() {
                    if (mEvent.is(Event::ReceiveComplete)) {
                        protocol_adapter::readReply();
                    }
                }
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
                        else {
                            mStateTick.on(retryTicks, []{
                                mState = State::Init;
                            });
                        }
                        break;
                    case State::Run:
                        mStateTick.on(cycleTicks, []{
                            mState = State::SendThrot;
                        });
                        break;
                    case State::GetValues:
                        if (mEvent.is(Event::OK)) {
                            mState = State::Run;
                        }
                        break;
                    case State::SendThrot:
                        mStateTick.on(cycleTicks, []{
                            mState = State::GetValues;
                        });
                        break;
                    }
                    if (oldState != mState) {
                        mStateTick.reset();
                        switch(mState) {
                        case State::Init:
                            IO::outl<debug>("# Vesc init");
                            break;
                        case State::Run:
                            // IO::outl<debug>("# Vesc run");
                            break;
                        case State::GetVersion:
                            IO::outl<debug>("# Vesc ver");
                            getVersion();
                            break;
                        case State::GetValues:
                            // IO::outl<debug>("# Vesc val");
                            getValues();
                            break;
                        case State::SendThrot:
                            // IO::outl<debug>("# Vesc throt");
                            sendThrottle();
                            break;
                        }
                    }
                }
                public:
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
                    static inline void onTransferComplete(const auto f) {
                        auto fEnable = [&]{
                            f();
                            uart::template rxEnable<true>();
                        };
                        if (mActive) {
                            uart::Isr::onTransferComplete(fEnable);
                        }
                    }
                };
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
                static inline void sendThrottle() {
                    char* const data = (char*)uart::outputBuffer();
                    CRC16 cs;
                    uint8_t n = 0;
                    data[n++] = 0x02;
                    data[n++] = 0x05;
                    cs += data[n++] = (uint8_t)CommPacketId::COMM_SET_DUTY;
                    cs += data[n++] = mThrottle >> 24;
                    cs += data[n++] = mThrottle >> 16;
                    cs += data[n++] = mThrottle >> 8;
                    cs += data[n++] = mThrottle;
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
                static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t size) {
                    if (data[0] != 0x02) {
                        return false;
                    }
                    const uint8_t length = data[1];
                    const uint8_t totalLength = length + 2 + 2 + 1;
                    if (totalLength > size) {
                        return false;
                    }
                    if (data[length + 4] != 0x03) {
                        return false;
                    }
                    return true;
                }
                static inline int32_t sbus2throt(const uint16_t sbus) {
                    int v = (sbus - 992) * 100'000 / 820;
                    return std::clamp(v, -100'000, 100'000);
                }
                static inline void send(const uint8_t n) {
                    uart::startSend(n);
                }
                static inline void deviceInfo(const DeviceInfo& data) {
                    mVersionMajor = data.mVersionMajor;
                    mVersionMinor = data.mVersionMinor;
                }
                static inline void telemetry(const TelemetryValues& data) {
                    if (mUseMotorCurrent) {
                        mCurrent = std::abs(data.mCurrent);
                    }
                    else {
                        mCurrent = std::abs(data.mCurrentIn);
                    }
                    mRpm = std::abs(data.mRpm) / mPolePairs;
                }
                template<typename CB>
                struct ProtocolAdapter {
                    enum class State : uint8_t { Idle, Length, Type,
                                                 Version, Version_Min, Version_Name, Version_UUID, Version_Pair, Version_Test, Version_HW, Version_Remain,
                                                 Values_Temp, Values_TempM, Values_CurM, Values_CurIn, Values_Rpm, Values_V, Values_Cons, Values_F, Values_Reamin,
                                                 CrcH, CrcL, End};

#if 0
                    // wrong: gaps are not considered (compare: readReply)
                    static inline void process(const uint8_t b) {
                        Debug::Scoped<tp> tp;
                        static CRC16 csum;
                        static uint8_t idata = 0;
                        static CommPacketId lastFrame = CommPacketId::COMM_ALIVE;
                        switch (mState) {
                        case State::Idle:
                            if (b == 0x02) {
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
                            csum += b;
                            --mLength;
                            if (const CommPacketId type = CommPacketId(b); type == CommPacketId::COMM_FW_VERSION) {
                                mState = State::Version;
                                lastFrame = type;
                            }
                            else if (type == CommPacketId::COMM_GET_VALUES) {
                                lastFrame = type;
                                mState = State::Values_Temp;
                                idata = 0;
                            }
                            else {
                                lastFrame = CommPacketId::COMM_ALIVE;
                                mState = State::Idle;
                            }
                            break;
                        case State::Version:
                            csum += b;
                            --mLength;
                            mDevInfo.mVersionMajor = b;
                            if (mLength == 0) {
                                mState = State::CrcH;
                            }
                            mState = State::Version_Min;
                            break;
                        case State::Version_Min:
                            csum += b;
                            --mLength;
                            mDevInfo.mVersionMinor = b;
                            if (mLength == 0) {
                                mState = State::CrcH;
                            }
                            mState = State::Version_Name;
                            idata= 0;
                            break;
                        case State::Version_Name:
                            csum += b;
                            --mLength;
                            if (idata < mDevInfo.mName.size()) {
                                mDevInfo.mName[idata++] = b;
                            }
                            if (b == '\0') {
                                mState = State::Version_UUID;
                                idata = 0;
                            }
                            if (mLength == 0) {
                                mState = State::CrcH;
                            }
                            break;
                        case State::Version_UUID:
                            csum += b;
                            --mLength;
                            if (++idata == 12) {
                                mState = State::Version_Pair;
                            }
                            if (mLength == 0) {
                                mState = State::CrcH;
                            }
                            break;
                        case State::Version_Pair:
                            csum += b;
                            --mLength;
                            if (mLength == 0) {
                                mState = State::CrcH;
                            }
                            mState = State::Version_Test;
                            break;
                        case State::Version_Test:
                            csum += b;
                            --mLength;
                            mDevInfo.mFWTestVersionNumber = b;
                            if (mLength == 0) {
                                mState = State::CrcH;
                            }
                            else {
                                mState = State::Version_HW;
                            }
                            break;
                        case State::Version_HW:
                            csum += b;
                            --mLength;
                            mDevInfo.mHWType = b;
                            if (mLength == 0) {
                                mState = State::CrcH;
                            }
                            else {
                                mState = State::Version_Remain;
                            }
                            break;
                        case State::Version_Remain:
                            csum += b;
                            --mLength;
                            if (mLength == 0) {
                                mState = State::CrcH;
                            }
                            break;
                        case State::Values_Temp:
                            csum += b;
                            --mLength;
                            if (idata == 0) {
                                mTelem.mTemperature = b;
                                ++idata;
                            }
                            else if (idata == 1) {
                                mTelem.mTemperature <<= 8;
                                mTelem.mTemperature |= b;
                                mState = State::Values_TempM;
                                idata = 0;
                            }
                            break;
                        case State::Values_TempM:
                            csum += b;
                            --mLength;
                            if (idata == 0) {
                                mTelem.mTemperatureMotor = b;
                                ++idata;
                            }
                            else if (idata == 1) {
                                mTelem.mTemperatureMotor <<= 8;
                                mTelem.mTemperatureMotor |= b;
                                mState = State::Values_CurM;
                                idata = 0;
                            }
                            break;
                        case State::Values_CurM:
                            csum += b;
                            --mLength;
                            if (idata == 0) {
                                mTelem.mCurrent = b;
                            }
                            else {
                                mTelem.mCurrent <<= 8;
                                mTelem.mCurrent |= b;
                            }
                            ++idata;
                            if (idata == 4) {
                                mState = State::Values_CurIn;
                                idata = 0;
                            }
                            break;
                        case State::Values_CurIn:
                            csum += b;
                            --mLength;
                            if (idata == 0) {
                                mTelem.mCurrentIn = b;
                            }
                            else {
                                mTelem.mCurrentIn <<= 8;
                                mTelem.mCurrentIn |= b;
                            }
                            ++idata;
                            if (idata == 4) {
                                mState = State::Values_Rpm;
                                idata = 0;
                            }
                            break;
                        case State::Values_Rpm:
                            csum += b;
                            --mLength;
                            if (idata == 0) {
                                mTelem.mRpm = b;
                            }
                            else {
                                mTelem.mRpm <<= 8;
                                mTelem.mRpm |= b;
                            }
                            ++idata;
                            if (idata == 4) {
                                mState = State::Values_V;
                                idata = 0;
                            }
                            break;
                        case State::Values_V:
                            csum += b;
                            --mLength;
                            if (idata == 0) {
                                mTelem.mVoltage = b;
                                ++idata;
                            }
                            else if (idata == 1) {
                                mTelem.mVoltage <<= 8;
                                mTelem.mVoltage |= b;
                                mState = State::Values_Cons;
                                idata = 0;
                            }
                            break;
                        case State::Values_Cons:
                            csum += b;
                            --mLength;
                            if (idata == 0) {
                                mTelem.mConsumption = b;
                            }
                            else {
                                mTelem.mConsumption <<= 8;
                                mTelem.mConsumption |= b;
                            }
                            ++idata;
                            if (idata == 4) {
                                mState = State::Values_F;
                                idata = 0;
                            }
                            break;
                        case State::Values_F:
                            csum += b;
                            --mLength;
                            mTelem.mFault = b;
                            if (mLength == 0) {
                                mState = State::CrcH;
                            }
                            else {
                                mState = State::Values_Reamin;
                            }
                            break;
                        case State::Values_Reamin:
                            csum += b;
                            --mLength;
                            if (mLength == 0) {
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
                                event(Event::OK);
                                ++mPackages;
                                if (lastFrame == CommPacketId::COMM_FW_VERSION) {
                                    CB::deviceInfo(mDevInfo);
                                }
                                else if (lastFrame == CommPacketId::COMM_GET_VALUES) {
                                    CB::telemetry(mTelem);
                                }
                            }
                            else {
                                event(Event::Error);
                            }
                            mState = State::End;
                            break;
                        case State::End:
                            if (b == 0x03) {
                                mState = State::Idle;
                            }
                            mState = State::Idle;
                            break;
                        }
                    }
#endif
                    static inline void readReply() {
                        Debug::Scoped<tp> tp;
                        const char* const data = (char*)uart::readBuffer();
                        // const uint16_t nread = uart::readCount();

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
                        uint16_t vcs = (data[length + 2] << 8) | data[length + 3];

                        if (vcs != cs) {
                            event(Event::Error);
                            return;
                        }

                        uint16_t k = 3;
                        if (type == CommPacketId::COMM_FW_VERSION) {
                            mDevInfo.mVersionMajor = (uint8_t)data[k++];
                            mDevInfo.mVersionMinor = (uint8_t)data[k++];
                            for(uint16_t i = 0; i < mDevInfo.mName.size(); ++i) {
                                mDevInfo.mName[i] = (char)data[k++];
                                if (mDevInfo.mName[i] == '\0')
                                    break;
                            }
                            k += 12; // UUID
                            k++; // pairing
                            mDevInfo.mFWTestVersionNumber = (uint8_t)data[k++];
                            mDevInfo.mHWType = (uint8_t)data[k++]; // enum?
                            CB::deviceInfo(mDevInfo);
                        }
                        else if (type == CommPacketId::COMM_GET_VALUES) {
                            mTelem.mTemperature = ((int32_t)data[3]) << 8;
                            mTelem.mTemperature |= ((int32_t)data[4]);

                            mTelem.mTemperatureMotor = ((int32_t)data[5]) << 8;
                            mTelem.mTemperatureMotor |= ((int32_t)data[6]);

                            mTelem.mCurrent = ((int32_t)data[7]) << 24;
                            mTelem.mCurrent |= ((int32_t)data[8]) << 16;
                            mTelem.mCurrent |= ((int32_t)data[9]) << 8;
                            mTelem.mCurrent |= ((int32_t)data[10]);

                            if (mTelem.mCurrent < 0) {
                                mTelem.mCurrent = -mTelem.mCurrent;
                            }

                            mTelem.mCurrentIn = ((int32_t)data[11]) << 24;
                            mTelem.mCurrentIn |= ((int32_t)data[12]) << 16;
                            mTelem.mCurrentIn |= ((int32_t)data[13]) << 8;
                            mTelem.mCurrentIn |= ((int32_t)data[14]);

                            if (mTelem.mCurrentIn < 0) {
                                mTelem.mCurrentIn = -mTelem.mCurrentIn;
                            }

                            mTelem.mRpm = ((int32_t)data[25]) << 24;
                            mTelem.mRpm |= ((int32_t)data[26]) << 16;
                            mTelem.mRpm |= ((int32_t)data[27]) << 8;
                            mTelem.mRpm |= ((int32_t)data[28]);

                            if (mTelem.mRpm < 0) {
                                mTelem.mRpm = -mTelem.mRpm;
                            }

                            mTelem.mVoltage = ((int32_t)data[29]) << 8;
                            mTelem.mVoltage |= ((int32_t)data[30]);

                            mTelem.mConsumption = ((int32_t)data[31]) << 24;
                            mTelem.mConsumption |= ((int32_t)data[32]) << 16;
                            mTelem.mConsumption |= ((int32_t)data[33]) << 8;
                            mTelem.mConsumption |= ((int32_t)data[34]);

                            mTelem.mFault = (uint8_t)data[55];
                            CB::telemetry(mTelem);
                        }
                        event(Event::OK);
                    }
                    private:
                    static inline uint16_t mCRC{};
                    static inline State mState = State::Idle;
                    static inline uint8_t mLength{};
                    static inline uint16_t mPackages{};
                    static inline TelemetryValues mTelem;
                    static inline DeviceInfo mDevInfo;
                };
                struct UartConfig {
                    using Clock = clock;
                    using ValueType = uint8_t;
                    using DmaChComponent = dmaChComponent;
                    using Adapter = void;
                    static inline constexpr bool fifo = true;
                    static inline constexpr Mcu::Stm::Uarts::Mode mode = Mcu::Stm::Uarts::Mode::HalfDuplex;
                    static inline constexpr uint32_t baudrate = 115'200;
                    struct Rx {
                        static inline constexpr bool enable = false;
                        static inline constexpr size_t size = 256;
                        static inline constexpr size_t idleMinSize = 8;
                    };
                    struct Tx {
                        static inline constexpr bool enable = true;
                        static inline constexpr size_t size = 64;
                    };
                    struct Isr {
                        static inline constexpr bool idle = true;
                        static inline constexpr bool txComplete = true;
                    };
                    using tp = Serial::tp;
                };
                static inline uint8_t mVersionMajor;
                static inline uint8_t mVersionMinor;
                static inline uint16_t mRpm{};
                static inline uint8_t mPolePairs{7};
                static inline bool mUseMotorCurrent = false;
                static inline uint16_t mCurrent{};
                static inline int32_t mThrottle{};
                static inline External::Tick<systemTimer> mStateTick;
                static inline volatile etl::Event<Event> mEvent;
                static inline volatile State mState = State::Init;
                static inline volatile bool mActive = false;
            };
        }
    }
}