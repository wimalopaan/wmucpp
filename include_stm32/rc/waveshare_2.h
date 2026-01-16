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

#include "mcu/alternate.h"

#include "usart_2.h"
#include "output.h"
#include "etl/util.h"
#include "debug_pin.h"

using namespace std::literals::chrono_literals;

namespace External::WaveShare {
    namespace V2 {
        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct Servo {
            using polar = Config::polar;
            using debug = Config::dbg;
            using tp = Config::tp;
            using dmaChComponent = Config::dmaChComponent;
            using systemTimer = Config::timer;
            using storage = Config::storage;

            struct UartConfig {
                using Clock = Config::clk;
                using ValueType = uint8_t;
                using DmaChComponent = dmaChComponent;
                static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::HalfDuplex;
                static inline constexpr uint32_t baudrate = 1'000'000;
                struct Rx {
                    static inline constexpr size_t size = 16;
                    static inline constexpr size_t idleMinSize = 4;
                };
                struct Tx {
                    static inline constexpr bool singleBuffer = true;
                    static inline constexpr bool enable = true;
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
                using pin = Config::pin;
                static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;
                IO::outl<debug>("# WS init");
                Mcu::Arm::Atomic::access([]{
                    mState = State::Init;
                    mEvent = Event::None;
                    mReadErrorCount = 0;
                    mReceivedPackets = 0;
                    mLastError = 0;
                    mStateTick.reset();
                    mActive = true;
                    uart::init();
                });
                pin::afunction(af);
                pin::template pullup<true>();
            }
            static inline void reset() {
                IO::outl<debug>("# WS reset");
                Mcu::Arm::Atomic::access([]{
                    uart::reset();
                    mActive = false;
                });
            }
            static inline constexpr External::Tick<systemTimer> initTicks{3000ms};
            static inline constexpr External::Tick<systemTimer> stepTicks{50ms};
            static inline constexpr External::Tick<systemTimer> telemTicks{10ms};

            static inline constexpr uint8_t CmdPing  = 0x01;
            static inline constexpr uint8_t CmdRead  = 0x02;
            static inline constexpr uint8_t CmdWrite = 0x03;
            static inline constexpr uint8_t CmdReset = 0x06;

            enum class Event : uint8_t {None, ReadReply, ResetServo};
            enum class State : uint8_t {Init,
                                        ResetServo,
                                        ReadFwHw,
                                        SetAbsoluteMode,
                                        SetMinMax,
                                        SetDisableAutoReply,
                                        ReadPosition,
                                        SetPosition};
            enum class ServoCommand : uint8_t {None, ReadFwHw, ReadPosition};

            static inline void speed(const uint16_t s) {
                IO::outl<debug>("# WS speed ", s);
                mSpeed = std::min(s, uint16_t{3400});
            }
            static inline void event(const Event e) {
                mEvent = e;
            }
            static inline void zero() {
                event(Event::ResetServo);
            }
            static inline void update() {
                mPhi = polar::phi();
            }
            static inline void offset(const uint16_t o) {
                mOffset = etl::normalize<4096>(o);
            }
            static inline uint16_t offset() {
                return mOffset;
            }
            static inline int8_t turns() {
                return mTurns;
            }
            static inline int16_t absPos() {
                return (mTurns * 4096) + mLastPos;
            }
            static inline uint16_t speed() {
                return mSpeed;
            }
            static inline uint16_t lastPos() {
                return mLastPos;
            }
            static inline int16_t actualPos() {
                return mActualPos;
            }
            static inline uint16_t errorCount() {
                return mReadErrorCount;
            }
            static inline std::pair<uint8_t, uint8_t> hwVersion() {
                return mHW;
            }
            static inline std::pair<uint8_t, uint8_t> fwVersion() {
                return mFW;
            }
            struct Isr {
                static inline void onIdle(const auto f) {
                    if (mActive) {
                        const auto f2 = [&](const volatile uint8_t* const data, const uint16_t size){
                            f();
                            if (validityCheck(data, size)) {
                                event(Event::ReadReply);
                                return true;
                            }
                            return false;
                        };
                        uart::Isr::onIdle(f2);
                    }
                }
                static inline void onTransferComplete(const auto f) {
                    if (mActive) {
                        const auto fEnable = [&]{
                            f();
                            uart::template rxEnable<true>();
                        };
                        uart::Isr::onTransferComplete(fEnable);
                    }
                }
            };

            static inline void periodic() {
                switch(mState) {
                case State::Init:
                    break;
                case State::ReadFwHw:
                case State::ResetServo:
                case State::SetAbsoluteMode:
                case State::SetDisableAutoReply:
                case State::SetMinMax:
                case State::ReadPosition:
                case State::SetPosition:
                    if (mEvent.is(Event::ReadReply)) {
                        readReply();
                    }
                    break;
                }
            }
            static inline void ratePeriodic() {
                const auto oldState = mState;
                ++mStateTick;
                switch(mState) {
                case State::Init:
                    mStateTick.on(initTicks, []{
                       mState = State::ResetServo;
                    });
                    break;
                case State::ResetServo:
                    mStateTick.on(stepTicks, []{
                        mState = State::ReadFwHw;
                    });
                    break;
                case State::ReadFwHw:
                    mStateTick.on(stepTicks, []{
                        mState = State::SetAbsoluteMode;
                    });
                    break;
                case State::SetAbsoluteMode:
                    mStateTick.on(stepTicks, []{
                        mState = State::SetMinMax;
                    });
                    break;
                case State::SetMinMax:
                    mStateTick.on(stepTicks, []{
                        mState = State::SetDisableAutoReply;
                    });
                    break;
                case State::SetDisableAutoReply:
                    mStateTick.on(stepTicks, []{
                       mState = State::SetPosition;
                    });
                    break;
                case State::SetPosition:
                    if (mEvent.is(Event::ResetServo)) {
                        mState = State::ResetServo;
                    }
                    else {
                        mStateTick.on(telemTicks, []{
                            mState = State::ReadPosition;
                        });
                    }
                    break;
                case State::ReadPosition:
                    mStateTick.on(telemTicks, []{
                        mState = State::SetPosition;
                    });
                    break;
                }
                if (oldState != mState) {
                    mStateTick.reset();
                    switch(mState) {
                    case State::Init:
                        break;
                    case State::ResetServo:
                        mTurns = 0;
                        mPhi = 0;
                        mLastPos = 0;
                        resetServo();
                        break;
                    case State::ReadFwHw:
                        queryFwHw();
                        break;
                    case State::SetAbsoluteMode:
                        setAboluteMode();
                        break;
                    case State::SetMinMax:
                        setMinMax();
                        break;
                    case State::SetDisableAutoReply:
                        disableAutoReply();
                        break;
                    case State::SetPosition:
                        setPosition();
                        break;
                    case State::ReadPosition:
                        readPosition();
                        break;
                    }
                }
            }
            private:
            static inline void queryFwHw() {
                std::array<uint8_t, 1> payload;
                payload[0] = 0x04; // return len
                mLastServoCommand = ServoCommand::ReadFwHw;
                send(mId, 0x00, payload, CmdRead); // address 0
            }
            static inline void setAboluteMode() {
                std::array<uint8_t, 1> payload;
                payload[0] = 0x00; // absolute mode
                send(mId, 0x21, payload);
            }
            static inline void disableAutoReply() {
                std::array<uint8_t, 1> payload;
                payload[0] = 0x00; // not auto reply
                send(mId, 0x08, payload);
            }
            static inline void setMinMax() {
                std::array<uint8_t, 4> payload;
                payload[0] = 0x00; // unlimited
                payload[1] = 0x00;
                payload[2] = 0x00;
                payload[3] = 0x00;
                send(mId, 0x09, payload);
            }
            static inline void readPosition() {
                std::array<uint8_t, 1> payload;
                payload[0] = 0x02; // return len
                mLastServoCommand = ServoCommand::ReadPosition;
                send(mId, 0x38, payload, CmdRead); // address 0x38
            }
            static inline void setPosition() {
                std::array<uint8_t, 6> payload;
                const uint16_t po = etl::normalize<4096>(mPhi + mOffset);
                int16_t d = po - mLastPos;
                if (d >= 2048) {
                    mTurns -= 1;
                }
                else if (d <= -2048) {
                    mTurns += 1;
                }
                int32_t pa = std::clamp(po + mTurns * 4096, -6 * 4096, 6 * 4096);
                const uint16_t s = std::abs(pa);
                payload[2] = 0;
                payload[3] = 0;
                payload[4] = mSpeed;
                payload[5] = mSpeed >> 8;
                if (pa >= 0) {
                    payload[0] = s;
                    payload[1] = s >> 8;
                    send(mId, 0x2a, payload);
                }
                else if (pa < 0) {
                    payload[0] = s;
                    payload[1] = (s >> 8) | 0x80 ; // neg. direction
                    send(mId, 0x2a, payload);
                }
                mLastPos = po;
            }
            static inline uint8_t add(volatile uint8_t* const data, uint8_t& i, const uint8_t v) {
                data[i++] = v;
                return v;
            }
            static inline void send(const uint8_t id, const uint8_t address, const auto& payload,
                                    const uint8_t cmd = CmdWrite) {
                uart::fillSendBuffer([&](auto& data){
                    uint8_t csum = 0;
                    uint8_t i = 0;
                    etl::assign(data[i++], 0xff);
                    etl::assign(data[i++], 0xff);
                    csum += etl::assign(data[i++], id);
                    csum += etl::assign(data[i++], 1 + 1 + 1 + payload.size()); // len = 1(cmd) + 1(adr) + payload + 1(cs)
                    csum += etl::assign(data[i++], cmd);
                    csum += etl::assign(data[i++], address);
                    for(uint8_t p = 0; p < payload.size(); ++p) {
                        csum += etl::assign(data[i++], payload[p]);
                    }
                    etl::assign(data[i++], ~csum);
                    return i;
                });
                // volatile uint8_t* const data = uart::outputBuffer();
                // uint8_t csum = 0;
                // uint8_t i = 0;
                // add(data, i, 0xff);
                // add(data, i, 0xff);
                // csum += add(data, i, id);
                // csum += add(data, i, 1 + 1 + 1 + payload.size()); // len = 1(cmd) + 1(adr) + payload + 1(cs)
                // csum += add(data, i, cmd);
                // csum += add(data, i, address);
                // for(uint8_t p = 0; p < payload.size(); ++p) {
                //     csum += add(data, i, payload[p]);
                // }
                // add(data, i, ~csum);
                // uart::startSend(i);
            }
            static inline void resetServo() {
                uart::fillSendBuffer([](auto& data){
                    uint8_t csum = 0;
                    uint8_t i = 0;
                    etl::assign(data[i++], 0xff);
                    etl::assign(data[i++], 0xff);
                    csum += etl::assign(data[i++], mId);
                    csum += etl::assign(data[i++], 2); // len = 1(cmd) + 1(cs)
                    csum += etl::assign(data[i++], CmdReset);
                    etl::assign(data[i++], ~csum);
                    return i;
                });
                // volatile uint8_t* const data = uart::outputBuffer();
                // uint8_t csum = 0;
                // uint8_t i = 0;
                // add(data, i, 0xff);
                // add(data, i, 0xff);
                // csum += add(data, i, mId);
                // csum += add(data, i, 2); // len = 1(cmd) + 1(cs)
                // csum += add(data, i, CmdReset);
                // add(data, i, ~csum);
                // uart::startSend(i);
            }
            static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t size) {
                if (data[0] != 0xff) {
                    return false;
                }
                if (data[1] != 0xff) {
                    return false;
                }
                const uint8_t len = data[3];
                const uint8_t totalLength = len + 4;
                if (totalLength > size) {
                    return false;
                }
                return true;
            }
            static inline void readReply() {
                [[maybe_unused]] Debug::Scoped<tp> tp;

                uart::readBuffer([](const auto& data){
                    ++mReceivedPackets;
                    const uint8_t id = data[2];
                    const uint8_t len = data[3];
                    mLastError = data[4];
                    uint8_t csum = id + len + mLastError;
                    const uint8_t dataLength = len - 2;
                    for(uint8_t i = 0; i < dataLength; ++i) {
                        csum += data[5 + i];
                    }
                    if (data[3 + len] != (~csum & 0xff)) {
                        ++mReadErrorCount;
                    }
                    else {
                        if (const auto c = std::exchange(mLastServoCommand, ServoCommand::None); c == ServoCommand::ReadFwHw) {
                            if (dataLength == 4) {
                                mFW.first  = data[5];
                                mFW.second = data[6];
                                mHW.first  = data[7];
                                mHW.second = data[8];
                            }
                            else {
                                ++mReadErrorCount;
                            }
                        }
                        else if (c == ServoCommand::ReadPosition) {
                            if (dataLength == 2) {
                                const uint8_t lb = data[5];
                                const uint8_t hb = data[6];
                                int16_t p = 0;
                                if (hb & 0x80) {
                                    p = -(((hb & 0x7f) << 8) + lb);
                                }
                                else {
                                    p =  (((hb & 0x7f) << 8) + lb);
                                }
                                mActualPos = etl::normalize(p - mOffset);
                            }
                            else {
                                ++mReadErrorCount;
                            }
                        }
                        else {
                            ++mReadErrorCount;
                        }
                    }
                });

                // ++mReceivedPackets;
                // const uint8_t id = uart::readBuffer()[2];
                // const uint8_t len = uart::readBuffer()[3];
                // mLastError = uart::readBuffer()[4];
                // uint8_t csum = id + len + mLastError;
                // const uint8_t dataLength = len - 2;
                // for(uint8_t i = 0; i < dataLength; ++i) {
                //     csum += uart::readBuffer()[5 + i];
                // }
                // if (uart::readBuffer()[3 + len] != (~csum & 0xff)) {
                //     ++mReadErrorCount;
                // }
                // else {
                //     if (const auto c = std::exchange(mLastServoCommand, ServoCommand::None); c == ServoCommand::ReadFwHw) {
                //         if (dataLength == 4) {
                //             mFW.first  = uart::readBuffer()[5];
                //             mFW.second = uart::readBuffer()[6];
                //             mHW.first  = uart::readBuffer()[7];
                //             mHW.second = uart::readBuffer()[8];
                //         }
                //         else {
                //             ++mReadErrorCount;
                //         }
                //     }
                //     else if (c == ServoCommand::ReadPosition) {
                //         if (dataLength == 2) {
                //             const uint8_t lb = uart::readBuffer()[5];
                //             const uint8_t hb = uart::readBuffer()[6];
                //             int16_t p = 0;
                //             if (hb & 0x80) {
                //                 p = -(((hb & 0x7f) << 8) + lb);
                //             }
                //             else {
                //                 p =  (((hb & 0x7f) << 8) + lb);
                //             }
                //             mActualPos = normalize(p - mOffset);
                //         }
                //         else {
                //             ++mReadErrorCount;
                //         }
                //     }
                //     else {
                //         ++mReadErrorCount;
                //     }
                // }
            }
            static inline bool mActive = false;
            static inline etl::Event<Event> mEvent;
            static inline State mState{State::Init};
            static inline ServoCommand mLastServoCommand = ServoCommand::None;
            static inline uint32_t mReadErrorCount = 0;
            static inline uint8_t mLastError = 0;
            static inline uint32_t mReceivedPackets = 0;
            static inline std::pair<uint8_t, uint8_t> mFW{};
            static inline std::pair<uint8_t, uint8_t> mHW{};
            static inline External::Tick<systemTimer> mStateTick;
            static inline uint8_t mId = 1;
            static inline uint16_t mLastPos = 0;
            static inline uint16_t mPhi = 0;
            static inline int16_t mActualPos = 0;
            static inline uint16_t mOffset = 0;
            static inline uint16_t mSpeed = 1000;
            static inline int8_t mTurns = 0;
        };
    }
}


