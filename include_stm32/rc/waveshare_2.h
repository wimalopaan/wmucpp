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

#include "mcu/alternate.h"

#include "usart_2.h"
#include "output.h"
#include "etl/util.h"
#include "etl/event.h"
#include "etl/fixedvector.h"
#include "etl/state.h"
#include "debug_pin.h"

using namespace std::literals::chrono_literals;

namespace External::WaveShare {
    namespace V4 {
        
        /*
         * adr  length
         * 
         * 
         * query actual status
         * 0x38 2       current position
         * 0x3a 2       current speed
         * 0x3c 2       current load
         * 0x3e 1       voltage
         * 0x3f 1       temperature
         * 0x40 1       asynch write flag
         * 0x41 1       status
         * 0x42 1       moving
         * 0x45 2       current current
         * -----------
         *      13
         *
         * query info      
         * 0x00 1       fw major
         * 0x01 1       fw minor
         * 0x02 1       ?
         * 0x03 1       hw major
         * 0x04 1       hw minor
         * -----------
         *      6
         * general settings
         * 0x09 2       min angle
         * 0x0b 2       max angle
         * 
         * 0x21 1       operation mode
         * 
         * 0x08 1       set response mode
         * 
         * 0x29 1       acceleration
         * 
         * 0x2e 2       speed
         *
         * operation 
         * 0x2a 2       target position
         * 
         * one-time settings
         * 0x05 1       ID
         * 
         * 0x37 1       eeprom lock
        */
        
        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct Servo {
            using polar = Config::polar;
            using debug = Config::dbg;
            using tp = Config::tp;
            using dmaChComponent = Config::dmaChComponent;
            using systemTimer = Config::timer;
            using storage = Config::storage;
            using pin = Config::pin;
            using callback = Config::callback;

            static inline constexpr uint8_t MaxServos = 16;
            static inline constexpr uint8_t MaxResponseSize = 32;

            struct UartConfig {
                using Clock = Config::clk;
                using ValueType = uint8_t;
                using DmaChComponent = dmaChComponent;
                static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::HalfDuplex;
                static inline constexpr uint32_t baudrate = 1'000'000;
                struct Rx {
                    static inline constexpr size_t size = MaxResponseSize;  
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
                using tp = void;
            };

            using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;

            static inline void init() {
                static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;
                IO::outl<debug>("# WS init");
                mReadErrorCount = 0;
                mReceivedPackets = 0;
                etl::fill(mLastError, 0);
                etl::fill(mFW, std::pair<uint8_t, uint8_t>{0, 0});
                etl::fill(mHW, std::pair<uint8_t, uint8_t>{0, 0});
                mServoIDs.clear();
                Mcu::Arm::Atomic::access([]{
                    mState = State::Init;
                    mEvent = Event::None;
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
                pin::analog();
            }
            static inline constexpr External::Tick<systemTimer> initTicks{1000ms};
            static inline constexpr External::Tick<systemTimer> stepTicks{10ms};
            static inline constexpr External::Tick<systemTimer> waitTicks{2ms}; // per servo
            static inline constexpr External::Tick<systemTimer> telemTicks{1ms};

            static inline constexpr uint8_t CmdPing      = 0x01;
            static inline constexpr uint8_t CmdRead      = 0x02;
            static inline constexpr uint8_t CmdWrite     = 0x03;
            static inline constexpr uint8_t CmdReset     = 0x06;
            static inline constexpr uint8_t CmdSyncRead  = 0x82;
            static inline constexpr uint8_t CmdSyncWrite = 0x83;

            static inline constexpr uint8_t broadcastId = 0xfe;
            
            enum class Event : uint8_t {None,
                                        SetPosition,
                                        SetSpeed,
                                        SetTorque,
                                        StartPing, StopPing,
                                        SetId
                                       };
            enum class State : uint8_t {None,
                                        Init,
                                        SetReplayMode,
                                        StartPing, Ping,
                                        ReadStatus,
                                        ReadFwHw,
                                        SetOperationMode,
                                        SetLimits,
                                        SetSpeed,
                                        SetTorque,
                                        SetPosition,
                                        WaitResponse,
                                        Run,
                                        SetId, 
                                        Unlock, 
                                        Lock
                                       };
            enum class ServoCommand : uint8_t {None, 
                                               Ping,
                                               SyncReadStatus,
                                               ReadFwHw};

            static inline void startPing() {
                mEvent = Event::StartPing;
            }
            static inline void stopPing() {
                mEvent = Event::StopPing;
            }
            static inline bool isPinging() {
                return mState.contains(State::StartPing, State::Ping);
            }
            static inline void setId(const uint8_t id) {
                mEvent = Event::SetId;
                mIdToBeSet = id;
            }
            static inline void speed(const uint8_t s, const uint16_t speed) {
                IO::outl<debug>("# WS speed[", s, "]: ", speed);
                mSpeed[s] = std::min(speed, uint16_t{3400});
                mEvent = Event::SetSpeed;
            }
            static inline void torque(const uint8_t s, const uint16_t t) {
                IO::outl<debug>("# WS torgue[", s, "]: ", t);
                mTorque[s] = std::min(t, uint16_t{1000});
                mEvent = Event::SetTorque;
            }
            static inline void gear(const uint8_t s, const uint16_t g) {
                IO::outl<debug>("# WS gear[", s, "]: ", g);
                mGearPercent[s] = std::min(g, uint16_t{600});
            }
            static inline void zero() {
                mEvent = Event::ResetServo;
            }
            // static inline void update() {
            //     mCircularMode = true;
            //     mPhi = polar::phi();
            // }
            static inline void set(const uint8_t s, const uint16_t sbus) {
                mCircularMode[s] = false;
                mLastPhi[s] = mPhi[s];
                mPhi[s] = sbus2pos(s, sbus);
                mEvent = Event::SetPosition;
                const uint16_t d = std::abs(mLastPhi[s] - mPhi[s]);
                if (d > 0) {
                    callback::onStart(s, d);
                }
                else {
                    callback::onStop(s);                    
                }
            }
            static inline void offset(const uint8_t s, const uint16_t o) {
                mOffset[s] = etl::normalize<4096>(o);
            }
            static inline uint16_t offset(const uint8_t s) {
                return mOffset[s];
            }
            static inline int8_t turns(const uint8_t s) {
                return mTurns[s];
            }
            static inline uint16_t absPhiDiff(const uint8_t s) {
                uint16_t d = etl::normalize(mActualPos[s] - (mPhi[s] + mOffset[s]));
                if (d >= 2048) {
                    d = 4096 - d;
                }           
                return d;
            }
            static inline int16_t absPos(const uint8_t s) {
                return (mTurns[s] * 4096) + mLastPos[s];
            }
            static inline uint16_t speed(const uint8_t s) {
                return mSpeed[s];
            }
            static inline uint16_t lastPos(const uint8_t s) {
                return mLastPos[s];
            }
            static inline int16_t actualPos(const uint8_t s) {
                return mActualPos[s];
            }
            static inline int16_t actualLoad(const uint8_t s) {
                return mActualLoad[s];
            }
            static inline int16_t actualCurrent(const uint8_t s) {
                return mActualCurrent[s];
            }
            static inline int16_t actualVoltage(const uint8_t s) {
                return mActualVoltage[s];
            }
            static inline int16_t actualSpeed(const uint8_t s) {
                return mActualSpeed[s];
            }
            static inline bool isMoving(const uint8_t s) {
                return mActualMoving[s];
            }   
            static inline uint16_t errorCount() {
                return mReadErrorCount;
            }
            static inline uint32_t packets(const uint8_t s) {
                return mReceivedServoPackets[s];
            }
            static inline std::pair<uint8_t, uint8_t> hwVersion(const uint8_t s) {
                return mHW[s];
            }
            static inline std::pair<uint8_t, uint8_t> fwVersion(const uint8_t s) {
                return mFW[s];
            }
            static inline const auto& servoIds() {
                return mServoIDs;
            }
            struct Isr {
                static inline void onIdle(const auto f) {
                    if (mActive) {
                        const auto f2 = [&](const volatile uint8_t* const data, const uint16_t size){
                            f();
                            checkedRead(data, size);
                            return true;
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
            static inline void ratePeriodic() {
                if (!mActive) {
                    return;
                }
                const auto oldState = mState;
                ++mStateTick;
                switch(mState) {
                case State::None:
                    break;
                case State::Init:
                    mStateTick.on(initTicks, []{
                        mState = State::SetReplayMode;
                    });
                    break;
                case State::SetReplayMode:
                    mStateTick.on(stepTicks, []{
                        mState = State::StartPing;
                    });
                    break;
                case State::StartPing:
                    mState = State::Ping;
                    break;
                case State::Ping:
                    // mEvent.on(Event::StartPing, []{
                    //     mState = State::Run;
                    // });
                    if (mActualPingId < mMaxPingID) {
                        (mState = State::WaitResponse) += State::Ping;
                    }
                    else {
                        callback::onPing();
                        mState = State::ReadFwHw;
                    }
                    break;
                case State::ReadFwHw:
                    (mState = State::WaitResponse) += State::SetOperationMode;
                    break;
                case State::SetOperationMode:
                    mStateTick.on(stepTicks, []{
                        mState = State::SetLimits;
                    });
                    break;
                case State::SetLimits:
                    mStateTick.on(stepTicks, []{
                        mState = State::SetSpeed;
                    });
                    break;
                case State::SetSpeed:
                    mStateTick.on(stepTicks, []{
                        mState = State::Run;
                    });
                    break;
                case State::SetTorque:
                    mStateTick.on(stepTicks, []{
                        mState = State::Run;
                    });
                    break;
                case State::Run:
                    mEvent.on(Event::StartPing, []{
                        mState = State::StartPing;
                    }).thenOn(Event::SetId, []{
                        (mState = State::Unlock) += State::SetId; 
                    }).thenOn(Event::SetPosition, []{
                        mState = State::SetPosition;
                    }).thenOn(Event::SetSpeed, []{
                        mState = State::SetSpeed;
                    }).thenOn(Event::SetTorque, []{
                        mState = State::SetTorque;
                    });
                    mStateTick.on(telemTicks, []{
                        mState = State::ReadStatus;
                    });
                    break;
                case State::SetPosition:
                    mStateTick.on(telemTicks, []{
                        mState = State::Run;
                    });
                    break;
                case State::ReadStatus:
                    (mState = State::WaitResponse) += State::Run;
                    break;
                case State::Unlock:
                    mStateTick.on(stepTicks, []{
                        mState = mState.following();
                    });
                    break;
                case State::SetId:
                    mStateTick.on(stepTicks, []{
                        (mState = State::Lock) += State::Run;
                    });
                    break;
                case State::Lock:
                    mStateTick.on(stepTicks, []{
                        mState = mState.following();
                    });
                    break;
                case State::WaitResponse:
                    mStateTick.on(waitTicks * std::max(mServoIDs.size(), uint8_t{1}), []{
                        if (mLastServoCommand == ServoCommand::SyncReadStatus) {
                            callback::onStatus();
                        }
                        mLastServoCommand = ServoCommand::None;
                        mState = mState.following();
                    });
                    break;
                }
                if (oldState != mState) {
                    mStateTick.reset();
                    switch(mState) {
                    case State::None:
                    case State::Init:
                        IO::outl<debug>("# WS init");
                        break;
                    case State::SetReplayMode:
                        IO::outl<debug>("# WS set reply");
                        disableAutoReply(broadcastId);
                        break;
                    case State::SetOperationMode:
                        IO::outl<debug>("# WS set op mode");
                        setAbsoluteMode(broadcastId);
                        break;
                    case State::SetLimits:
                        IO::outl<debug>("# WS limits");
                        setMinMax(broadcastId);
                        break;
                    case State::SetSpeed:
                        IO::outl<debug>("# WS speed");
                        setSpeed();
                        break;
                    case State::SetTorque:
                        IO::outl<debug>("# WS torque();");
                        setTorque();
                        break;
                    case State::StartPing:
                        IO::outl<debug>("# WS startPing");
                        mActualPingId = mMinPingID;
                        mServoIDs.clear();
                        break;
                    case State::Ping:
                        IO::outl<debug>("# WS ping");
                        sendPing(mActualPingId++);
                        break;
                    case State::Unlock:
                        IO::outl<debug>("# WS unlock");
                        unlock(broadcastId);
                        break;
                    case State::Lock:
                        IO::outl<debug>("# WS lock");
                        lock(broadcastId);
                        break;
                    case State::SetId:
                        IO::outl<debug>("# WS setid");
                        setServoId(broadcastId, mIdToBeSet);
                        break;
                    case State::Run:
                        break;
                    case State::SetPosition:
                        setPositionLinear();
                        break;
                    case State::ReadStatus:
                        syncReadStatus();
                        break;
                    case State::ReadFwHw:
                        IO::outl<debug>("# WS ReadFwHw");
                        syncQueryFwHw();
                        break;
                    case State::WaitResponse:
                        break;
                    }
                }
            }
            private:
            static inline void unlock(const uint8_t id) {
                std::array<uint8_t, 1> payload;
                payload[0] = 0x00; // nolock
                send(id, 0x37, payload, CmdWrite);
            }
            static inline void lock(const uint8_t id) {
                std::array<uint8_t, 1> payload;
                payload[0] = 0x01; // lock
                send(id, 0x37, payload, CmdWrite);                
            }
            static inline void setServoId(const uint8_t id, const uint8_t newId) {
                std::array<uint8_t, 1> payload;
                payload[0] = newId;
                send(id, 0x05, payload, CmdWrite);                
            }
            static inline void syncQueryFwHw() {
                mLastServoCommand = ServoCommand::ReadFwHw;
                queryAllIds(0x00, 5); // address 0x00
            }
            static inline void setAbsoluteMode(const uint8_t id) {
                std::array<uint8_t, 1> payload;
                payload[0] = 0x00; 
                send(id, 0x21, payload);                
            }
            static inline void disableAutoReply(const uint8_t id) {
                std::array<uint8_t, 1> payload;
                payload[0] = 0x00; // not auto reply
                send(id, 0x08, payload);
            }
            static inline void setMinMax(const uint8_t id) {
                std::array<uint8_t, 4> payload;
                payload[0] = 0x00; // unlimited
                payload[1] = 0x00;
                payload[2] = 0x00;
                payload[3] = 0x00;
                send(id, 0x09, payload);
            }
            static inline void setSpeed() {
                static std::array<std::array<uint8_t, 2>, MaxServos> payload; // not on stack
                for(uint8_t i = 0; i < mServoIDs.size(); ++i) {
                    payload[i][0] = mSpeed[i];
                    payload[i][1] = mSpeed[i] >> 8;
                }
                writeAllIds(0x2e, payload); 
            }
            static inline void setTorque() {
                static std::array<std::array<uint8_t, 2>, MaxServos> payload; // not on stack
                for(uint8_t i = 0; i < mServoIDs.size(); ++i) {
                    payload[i][0] = mTorque[i];
                    payload[i][1] = mTorque[i] >> 8;
                }
                writeAllIds(0x30, payload); 
            }
            static inline void syncReadStatus() {
                const uint8_t length = 13; // return len
                mLastServoCommand = ServoCommand::SyncReadStatus;
                queryAllIds(0x38, length); // address 0x38
            }
            static inline int32_t sbus2pos(const uint8_t s, const uint16_t sbus) {
                int32_t v = ((sbus - int32_t{992}) * 4096 * mGearPercent[s]) / (820 * 100);
                return v;
            }
            static inline void setPositionLinear() { // not reentrant
                static std::array<std::array<uint8_t, 2>, MaxServos> payload; // not on stack
                for(uint8_t i = 0; i < mServoIDs.size(); ++i) {
                    int32_t pa = std::clamp(mPhi[i], int16_t{-6 * 4096}, int16_t{6 * 4096});
                    const uint16_t s = std::abs(pa);
                    if (pa >= 0) {
                        payload[i][0] = s;
                        payload[i][1] = s >> 8;
                    }
                    else if (pa < 0) {
                        payload[i][0] = s;
                        payload[i][1] = (s >> 8) | 0x80 ; // neg. direction
                    }
                }
                writeAllIds(0x2a, payload); 
            }
            // static inline void setPosition() {
            //     std::array<uint8_t, 6> payload;
            //     const uint16_t po = etl::normalize<4096>(mPhi + mOffset);
            //     int16_t d = po - mLastPos;
            //     if (d >= 2048) {
            //         mTurns -= 1;
            //     }
            //     else if (d <= -2048) {
            //         mTurns += 1;
            //     }
            //     int32_t pa = std::clamp(po + mTurns * 4096, -6 * 4096, 6 * 4096);
            //     const uint16_t s = std::abs(pa);
            //     payload[2] = 0;
            //     payload[3] = 0;
            //     payload[4] = mSpeed;
            //     payload[5] = mSpeed >> 8;
            //     if (pa >= 0) {
            //         payload[0] = s;
            //         payload[1] = s >> 8;
            //         send(mId, 0x2a, payload);
            //     }
            //     else if (pa < 0) {
            //         payload[0] = s;
            //         payload[1] = (s >> 8) | 0x80 ; // neg. direction
            //         send(mId, 0x2a, payload);
            //     }
            //     mLastPos = po;
            // }
            static inline void sendPing(const uint8_t id) {
                mLastServoCommand = ServoCommand::Ping;
                uart::fillSendBuffer([&](auto& data){
                    uint8_t csum = 0;
                    uint8_t i = 0;
                    etl::assign(data[i++], 0xff);
                    etl::assign(data[i++], 0xff);
                    csum += etl::assign(data[i++], id);
                    csum += etl::assign(data[i++], 2); 
                    csum += etl::assign(data[i++], CmdPing);
                    etl::assign(data[i++], ~csum);
                    return i;
                });
            }
            template<auto L>
            static inline void send(const uint8_t id, const uint8_t address, const std::array<uint8_t, L>& payload,
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
            }
            static inline void queryAllIds(const uint8_t address, const uint8_t len) {
                uart::fillSendBuffer([&](auto& data){
                    uint8_t csum = 0;
                    uint8_t i = 0;
                    etl::assign(data[i++], 0xff);
                    etl::assign(data[i++], 0xff);
                    csum += etl::assign(data[i++], broadcastId);
                    csum += etl::assign(data[i++], 1 + 1 + 1 + 1 + mServoIDs.size()); // len = 1(cmd) + 1(adr) + 1(len) #id + 1(cs)
                    csum += etl::assign(data[i++], CmdSyncRead);
                    csum += etl::assign(data[i++], address);
                    csum += etl::assign(data[i++], len);
                    for(uint8_t p = 0; p < mServoIDs.size(); ++p) {
                        csum += etl::assign(data[i++], mServoIDs[p]);
                    }
                    etl::assign(data[i++], ~csum);
                    return i;
                });
            }
            template<auto L>
            static inline void writeAllIds(const uint8_t address, const std::array<uint8_t, L>& pl) {
                uart::fillSendBuffer([&](auto& data){
                    uint8_t csum = 0;
                    uint8_t i = 0;
                    etl::assign(data[i++], 0xff);
                    etl::assign(data[i++], 0xff);
                    csum += etl::assign(data[i++], broadcastId);
                    csum += etl::assign(data[i++], 1 + 1 + 1 + 1 + mServoIDs.size() * (pl.size() + 1)); // len = 1(cmd) + 1(adr) + 1(len) + #id * (pl.size() + 1) + 1(cs)
                    csum += etl::assign(data[i++], CmdSyncWrite);
                    csum += etl::assign(data[i++], address);
                    csum += etl::assign(data[i++], pl.size());
                    for(uint8_t p = 0; p < mServoIDs.size(); ++p) {
                        csum += etl::assign(data[i++], mServoIDs[p]);
                        for(const auto& e: pl) {
                            csum += etl::assign(data[i++], e);
                        }
                    }
                    etl::assign(data[i++], ~csum);
                    return i;
                });
            }
            template<auto L>
            static inline void writeAllIds(const uint8_t address, const std::array<std::array<uint8_t, L>, MaxServos>& pl) {
                uart::fillSendBuffer([&](auto& data){
                    uint8_t csum = 0;
                    uint8_t i = 0;
                    etl::assign(data[i++], 0xff);
                    etl::assign(data[i++], 0xff);
                    csum += etl::assign(data[i++], broadcastId);
                    csum += etl::assign(data[i++], 1 + 1 + 1 + 1 + mServoIDs.size() * (L + 1)); // len = 1(cmd) + 1(adr) + 1(len) + #id * (pl.size() + 1) + 1(cs)
                    csum += etl::assign(data[i++], CmdSyncWrite);
                    csum += etl::assign(data[i++], address);
                    csum += etl::assign(data[i++], L);
                    for(uint8_t p = 0; p < mServoIDs.size(); ++p) {
                        csum += etl::assign(data[i++], mServoIDs[p]);
                        for(const auto& e: pl[p]) {
                            csum += etl::assign(data[i++], e);
                        }
                    }
                    etl::assign(data[i++], ~csum);
                    return i;
                });
            }
            // static inline void resetServo() {
            //     uart::fillSendBuffer([](auto& data){
            //         uint8_t csum = 0;
            //         uint8_t i = 0;
            //         etl::assign(data[i++], 0xff);
            //         etl::assign(data[i++], 0xff);
            //         csum += etl::assign(data[i++], mId);
            //         csum += etl::assign(data[i++], 2); // len = 1(cmd) + 1(cs)
            //         csum += etl::assign(data[i++], CmdReset);
            //         etl::assign(data[i++], ~csum);
            //         return i;
            //     });
            // }
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
            static inline void checkedRead(const volatile uint8_t* const data, const uint16_t size) {
                if (validityCheck(data, size)){
                    [[maybe_unused]] Debug::Scoped<tp> tp;
                    mReceivedPackets += 1;
                    const uint8_t id = data[2];
                    const uint8_t len = data[3];
                    const uint8_t error = data[4];
                    uint8_t csum = id + len + error;
                    const uint8_t dataLength = len - 2;
                    for(uint8_t i = 0; i < dataLength; ++i) {
                        csum += data[5 + i];
                    }
                    if (data[3 + len] != (~csum & 0xff)) {
                        mReadErrorCount += 1;
                    }
                    else {
                        if (mState.contains(State::Ping)) {
                            if (len == 2) {
                                mServoIDs.push_back(id);
                                const int8_t index = indexOf(id);
                                mLastError[index] = error;
                            }
                        }
                        else {
                            const int8_t index = indexOf(id);
                            if (index < 0) {
                                return;
                            }
                            mReceivedServoPackets[index] += 1;
                            if (mLastServoCommand == ServoCommand::ReadFwHw) {
                                if (dataLength == 5) {
                                    mFW[index].first  = data[5];
                                    mFW[index].second = data[6];
                                    mHW[index].first  = data[8];
                                    mHW[index].second = data[9];
                                }
                            }
                            else if (mLastServoCommand == ServoCommand::SyncReadStatus) {
                                if (dataLength == 13) {
                                    uint8_t offset = 5;
                                    offset += decodePosition(index, &data[offset]);
                                    offset += decodeSpeed(index, &data[offset]);
                                    offset += decodeLoad(index, &data[offset]);
                                    offset += decodeVoltage(index, &data[offset]);
                                    offset += decodeTemperature(index, &data[offset]);
                                    ++offset;  // omit
                                    offset += decodeStatus(index, &data[offset]);
                                    offset += decodeMoving(index, &data[offset]);
                                    ++offset; // gap
                                    ++offset;
                                    offset += decodeCurrent(index, &data[offset]);
                                }
                            }
                        }
                    }
                }
            }
            static inline int8_t indexOf(const uint8_t id) {
                for(int8_t i = 0; i < mServoIDs.size(); ++i) {
                    if (id == mServoIDs[i]) {
                        return i;
                    }
                }
                return -1;
            }
            static inline uint8_t decodePosition(const uint8_t s, const volatile uint8_t* const data) {
                const uint8_t lb = data[0];
                const uint8_t hb = data[1];
                int16_t p = ((hb << 8) | lb) & 0xfff ;
                mActualPos[s] = etl::normalize(p - mOffset[s]);
                return 2;
            }
            static inline uint8_t decodeSpeed(const uint8_t s, const volatile uint8_t* const data) {
                const uint8_t lb = data[0];
                const uint8_t hb = data[1];
                uint16_t p = (hb << 8) | lb;
                if (p & 0x400) {
                    mActualSpeed[s] = -(p & 0x3ff);
                }
                else {
                    mActualSpeed[s] =  (p & 0x3ff);
                }
                return 2;
            }
            static inline uint8_t decodeLoad(const uint8_t s, const volatile uint8_t* const data) {
                const uint8_t lb = data[0];
                const uint8_t hb = data[1];
                uint16_t p = (hb << 8) | lb;
                if (p & 0x400) {
                    mActualLoad[s] = -(p & 0x3ff);
                }
                else {
                    mActualLoad[s] =  (p & 0x3ff);
                }
                return 2;
            }
            static inline uint8_t decodeVoltage(const uint8_t s, const volatile uint8_t* const data) {
                mActualVoltage[s] = data[0];
                return 1;
            }
            static inline uint8_t decodeTemperature(const uint8_t s, const volatile uint8_t* const data) {
                mActualTemperature[s] = data[0];
                return 1;
            }
            static inline uint8_t decodeStatus(const uint8_t s, const volatile uint8_t* const data) {
                mActualStatus[s] = data[0];
                return 1;
            }
            static inline uint8_t decodeMoving(const uint8_t s, const volatile uint8_t* const data) {
                mActualMoving[s] = (data[0] > 0);
                return 1;
            }
            static inline uint8_t decodeCurrent(const uint8_t s, const volatile uint8_t* const data) {
                const uint8_t lb = data[0];
                const uint8_t hb = data[1];
                uint16_t p = (hb << 8) + lb;
                mActualCurrent[s] = p;
                return 2;
            }
            static inline bool mActive = false;
            static inline etl::SlotEvent<Event> mEvent;
            static inline etl::State<volatile State> mState;

            static inline uint8_t mActualPingId = 0;
            static inline uint8_t mMinPingID = 1;
            static inline uint8_t mMaxPingID = 16;
            static inline uint8_t mIdToBeSet = 0;
            
            static inline volatile ServoCommand mLastServoCommand = ServoCommand::None;
            static inline etl::FixedVector<volatile uint8_t, MaxServos> mServoIDs{};
            static inline volatile uint32_t mReadErrorCount = 0;
            static inline volatile uint32_t mReceivedPackets = 0;
            static inline std::array<volatile uint32_t, MaxServos> mReceivedServoPackets{};
            static inline std::array<volatile uint8_t, MaxServos> mLastError{};
            static inline std::array<std::pair<volatile uint8_t, volatile uint8_t>, MaxServos> mFW{};
            static inline std::array<std::pair<volatile uint8_t, volatile uint8_t>, MaxServos> mHW{};

            static inline External::Tick<systemTimer> mStateTick;
            
            static inline std::array<uint16_t, MaxServos> mLastPos{};
            static inline std::array<int16_t,  MaxServos> mPhi{};
            static inline std::array<int16_t,  MaxServos> mLastPhi{};
            static inline std::array<int8_t, MaxServos>   mTurns{};
            
            static inline auto mGearPercent = etl::make_array_fill<std::array<uint16_t, MaxServos>>(100);
            static inline auto mOffset      = etl::make_array_fill<std::array<int16_t, MaxServos>>(0);
            static inline auto mSpeed       = etl::make_array_fill<std::array<uint16_t, MaxServos>>(3400);
            static inline auto mTorque      = etl::make_array_fill<std::array<uint16_t, MaxServos>>(1000);
            static inline std::array<bool, MaxServos> mCircularMode{};

            static inline std::array<volatile int16_t, MaxServos> mActualPos{};
            static inline std::array<volatile int16_t, MaxServos> mActualSpeed{};
            static inline std::array<volatile int16_t, MaxServos> mActualLoad{};
            static inline std::array<volatile int16_t, MaxServos> mActualCurrent{};
            static inline std::array<volatile uint8_t, MaxServos> mActualVoltage{};
            static inline std::array<volatile uint8_t, MaxServos> mActualTemperature{};
            static inline std::array<volatile uint8_t, MaxServos> mActualStatus{};
            static inline std::array<volatile uint8_t, MaxServos> mActualMoving{};
        };
    }
        
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
            enum class ServoCommand : uint8_t {None, ReadFwHw, ReadPosition, ReadLoad};

            static inline void speed(const uint16_t s) {
                IO::outl<debug>("# WS speed ", s);
                mSpeed = std::min(s, uint16_t{3400});
            }
            static inline void gear(const uint16_t s) {
                IO::outl<debug>("# WS gear", s);
                mGearPercent = std::min(s, uint16_t{600});
            }
            static inline void event(const Event e) {
                mEvent = e;
            }
            static inline void zero() {
                event(Event::ResetServo);
            }
            static inline void update() {
                mCircularMode = true;
                mPhi = polar::phi();
            }
            static inline void set(const uint16_t sbus) {
                mCircularMode = false;
                mPhi = sbus2pos(sbus);
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
                        if (mCircularMode) {
                            setPosition();
                        }
                        else {
                            setPositionLinear();
                        }
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
            static inline int32_t sbus2pos(const uint16_t sbus) {
                int32_t v = ((sbus - int32_t{992}) * 4096 * mGearPercent) / (820 * 100);
                return v;
            }
            static inline void setPositionLinear() {
                std::array<uint8_t, 6> payload;
                int32_t pa = std::clamp(mPhi, int16_t{-6 * 4096}, int16_t{6 * 4096});
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
                        else if (c == ServoCommand::ReadLoad) {
                            if (dataLength == 2) {
                                // const uint8_t lb = data[5];
                                // const uint8_t hb = data[6];
                                // int16_t p = (((hb & 0x7f) << 8) + lb);
                                // mActualLoad = p;
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
            static inline int16_t mPhi = 0;
            static inline uint16_t mGearPercent = 100;
            static inline int16_t mActualPos = 0;
            static inline uint16_t mOffset = 0;
            static inline uint16_t mSpeed = 1000;
            static inline int8_t mTurns = 0;
            static inline bool mCircularMode = true;
        };
    }
}


