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
#include <cmath>
#include <iterator>
#include <numbers>
#include <chrono>

#include "mcu/alternate.h"
#include "etl/algorithm.h"
#include "etl/ranged.h"
#include "etl/event.h"
#include "byte.h"
#include "units.h"
#include "tick.h"
#include "rc/rc_2.h"

namespace RC::Protokoll::ESCape {
    namespace V2 {
        using namespace std::literals::chrono_literals;

        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct ConfigAscii {
            using clock = Config::clock;
            using systemTimer = Config::systemTimer;
            using debug = Config::debug;
            using dmaChComponent = Config::dmaChComponent;
            using pin = Config::pin;
            using tp = Config::tp;

            struct UartConfig {
                using Clock = clock;
                using ValueType = uint8_t;
                using DmaChComponent = dmaChComponent;
                static inline constexpr Mcu::Stm::Uarts::Mode mode = Mcu::Stm::Uarts::Mode::HalfDuplex;
                static inline constexpr uint32_t baudrate = RC::Protokoll::ESCape::Ascii::baudrate;
                struct Rx {
                    static inline constexpr size_t size = 1024;
                    static inline constexpr size_t idleMinSize = 2;
                };
                struct Tx {
                    static inline constexpr bool singleBuffer = true;
                    static inline constexpr bool enable = true;
                    static inline constexpr size_t size = 64;
                };
                struct Isr {
                    static inline constexpr bool idle = true;
                    static inline constexpr bool txComplete = true;
                };
                using tp = Config::tp;
            };

            using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;

            static inline void init() {
                static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;
                IO::outl<debug>("# ESC32Ascii init");
                Mcu::Arm::Atomic::access([]{
                    uart::init();
                    mState = State::Init;
                    mEvent = Event::None;
                    mErrorCount = 0;
                    mActive = true;
                });
                pin::afunction(af);
                pin::template pullup<true>();
            }
            static inline void reset() {
                IO::outl<debug>("# ESC32Ascii reset");
                Mcu::Arm::Atomic::access([]{
                    mActive = false;
                    uart::reset();
                });
                pin::analog();
            }

            enum class State : uint8_t {Init, Music, Start, Show, Run, SendThrot, SendParam, Save, Beep};
            enum class Event : uint8_t {None, ReceiveComplete, OK, Error, SendThrottle, SendParam, Save, Beep};

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
                        mState = State::Music;
                    });
                    break;
                case State::Music:
                    if (mEvent.is(Event::OK)) {
                        mState = State::Show;
                    }
                    break;
                case State::Show:
                    if (mEvent.is(Event::OK)) {
                        mState = State::Start;
                    }
                    break;
                case State::Start:
                    if (mEvent.is(Event::OK)) {
                        mState = State::Run;
                    }
                    break;
                case State::Run:
                    if (mEvent.is(Event::SendThrottle)) {
                        mState = State::SendThrot;
                    }
                    else if (mEvent.is(Event::SendParam)) {
                        mState = State::SendParam;
                    }
                    else if (mEvent.is(Event::Save)) {
                        mState = State::Save;
                    }
                    else if (mEvent.is(Event::Beep)) {
                        mState = State::Beep;
                    }
                    break;
                case State::SendThrot:
                    if (mEvent.is(Event::OK)) {
                        mState = State::Run;
                    }
                    break;
                case State::SendParam:
                    if (mEvent.is(Event::OK)) {
                        mState = State::Run;
                    }
                    break;
                case State::Save:
                    if (mEvent.is(Event::OK)) {
                        mState = State::Run;
                    }
                    break;
                case State::Beep:
                    if (mEvent.is(Event::OK)) {
                        mState = State::Run;
                    }
                    else if (mEvent.is(Event::Error)) {
                        mState = State::Run;
                    }
                    break;
                }
                if (oldState != mState) {
                    mStateTick.reset();
                    switch(mState) {
                    case State::Init:
                        IO::outl<debug>("# ESC32Ascii init");
                        break;
                    case State::Music:
                        IO::outl<debug>("# ESC32Ascii music");
                        play("cegC_cegC_");
                        break;
                    case State::Show:
                        IO::outl<debug>("# ESC32Ascii show");
                        show();
                        break;
                    case State::Start:
                        IO::outl<debug>("# ESC32Ascii start");
                        mNextThrottle = 0;
                        throttle(0);
                        break;
                    case State::Run:
                        // IO::outl<debug>("# ESC32Ascii run");
                        break;
                    case State::SendThrot:
                        // IO::outl<debug>("# ESC32Ascii throt");
                        throttle(mNextThrottle);
                        break;
                    case State::SendParam:
                    {
                        IO::outl<debug>("# ESC32Ascii param");
                        uart::fillSendBuffer([](auto& data){
                            return snprintf((char*)&data[0], UartConfig::Tx::size, "set %s %d\n", mParams[mNextParam].name, mParams[mNextParam].value);
                        });
                        // char* const data = (char*)uart::outputBuffer();
                        // const uint8_t n = snprintf(data, UartConfig::Tx::size, "set %s %d\n", mParams[mNextParam].name, mParams[mNextParam].value);
                        // uart::startSend(n);
                    }
                        break;
                    case State::Save:
                    {
                        IO::outl<debug>("# ESC32Ascii save");
                        uart::fillSendBuffer([](auto& data){
                            return snprintf((char*)&data[0], UartConfig::Tx::size, "save\n");
                        });
                        // char* const data = (char*)uart::outputBuffer();
                        // const uint8_t n = snprintf(data, UartConfig::Tx::size, "save\n");
                        // uart::startSend(n);
                    }
                        break;
                    case State::Beep:
                        IO::outl<debug>("# ESC32Ascii beep");
                        play("cc_d_e_f_g_a_b#_CC");
                        break;
                    }
                }
            }
            static inline void set(const uint16_t sbus) {
                if (mActive && (mState == State::Run)) {
                    mNextThrottle = sbus2throt(sbus);
                    event(Event::SendThrottle);
                }
            }
            static inline void update() {
            }
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
                    if (mActive) {
                        auto fEnable = [&]{
                            f();
                            uart::template rxEnable<true>();
                        };
                        uart::Isr::onTransferComplete(fEnable);
                    }
                }
            };
            static inline std::pair<uint8_t, uint8_t> hwVersion() {
                return {};
            }
            static inline std::pair<uint8_t, uint8_t> fwVersion() {
                return {};
            }
            static inline auto errorCount() {
                return mErrorCount;
            }
            static inline uint16_t current() {
                return mCurrent;
            }
            static inline uint16_t rpm() {
                return mERT;
            }
            static inline void save() {
                if (mActive && (mState == State::Run)) {
                    event(Event::Save);
                }
            }
            static inline void beep() {
                if (mActive && (mState == State::Run)) {
                    event(Event::Beep);
                }
            }
            static inline void setParam(const uint8_t n, const uint16_t v) {
                if (mActive && (mState == State::Run)) {
                    if (n < mParams.size()) {
                        if (mParams[n].name) {
                            mParams[n].value = v;
                            mNextParam = n;
                            event(Event::SendParam);
                        }
                    }
                }
            }
            static inline constexpr auto& params() {
                return mParams;
            }
            private:
            static inline void throttle(const int throt) {
                uart::fillSendBuffer([&](auto& data){
                    return snprintf((char*)&data[0], UartConfig::Tx::size, "throt %d\n", throt);
                });
                // char* const data = (char*)uart::outputBuffer();
                // const uint8_t n = snprintf(data, UartConfig::Tx::size, "throt %d\n", throt);
                // uart::startSend(n);
            }
            static inline void show() {
                uart::fillSendBuffer([&](auto& data){
                    return snprintf((char*)&data[0], UartConfig::Tx::size, "show\n");
                });
                // char* const data = (char*)uart::outputBuffer();
                // const uint8_t n = snprintf(data, UartConfig::Tx::size, "show\n");
                // uart::startSend(n);
            }
            static inline void play(const char* const music) {
                uart::fillSendBuffer([&](auto& data){
                    return snprintf((char*)&data[0], UartConfig::Tx::size, "play %s\n", music);
                });
                // char* const data = (char*)uart::outputBuffer();
                // const uint8_t n = snprintf(data, UartConfig::Tx::size, "play %s\n", music);
                // uart::startSend(n);
            }
            static inline int sbus2throt(const uint16_t sbus) {
                const int t = ((sbus - RC::Protokoll::SBus::V2::mid) * RC::Protokoll::ESCape::span) / RC::Protokoll::SBus::V2::span;
                return std::clamp(t, RC::Protokoll::ESCape::min, RC::Protokoll::ESCape::max);
            }
            static inline bool validityCheck(const volatile uint8_t* const /*data*/, const uint16_t /*size*/) {
                return true;
            }
            static inline void readReply() {
                uart::readBuffer([](const auto& data){
                    uint16_t i = 0;
                    auto p = &data[0];
                    while(i < data.size()) {
                        if (data[i] == '\n') {
                            analyze((const char*)p, (const char*)&data[i]);
                            p = &data[i + 1];
                        }
                        ++i;
                    }
                });
                // const char* const data = (char*)uart::readBuffer();
                // const uint16_t l = uart::readCount();
                // uint16_t i = 0;
                // const char* p = data;
                // while(i < l) {
                //     if (data[i] == '\n') {
                //         analyze(p, &data[i]);
                //         p = &data[i + 1];
                //     }
                //     ++i;
                // }
            }
            static inline void gotOk() {
                // IO::outl<debug>("# OK");
                event(Event::OK);
            }
            static inline void gotError() {
                // IO::outl<debug>("# Error");
                event(Event::Error);
            }
            static inline void analyze(const char* const p, const char* const e) {
                if ((e - p) == 2) {
                    if (strncmp(p, "OK", 2) == 0) {
                        gotOk();
                        return;
                    }
                    else if (strncmp(p, "ok", 2) == 0) {
                        gotOk();
                        return;
                    }
                }
                else {
                    if ((e - p) == 5) {
                        if (strncmp(p, "ERROR", 5) == 0) {
                            gotError();
                            return;
                        }
                    }
                    for(uint16_t i = 0; (i < 64) && (&p[i] < e); ++i) {
                        if (p[i] == ':') {
                            value(p, i, e);
                        }
                    }
                }
            }
            // very ineffective
            static inline void value(const char* const p, const uint16_t d, const char* const e) {
                int v = 0;
                std::from_chars(p + d + 2, e, v);
                for(auto& param : mParams) {
                    if (strncmp(p, param.name, d) == 0) {
                        param.value = v;
                        // IO::outl<debug>("# value: ", param.name, " : ", param.value);
                    }
                }
            }
            template<typename T = uint16_t>
            struct Param {
                const char* const name = nullptr;
                T value = 0;
                const T min = 0;
                const T max = 1;
            };
#ifdef ESCAPE32_U8
            using param_t = Param<uint8_t>;
#else
            using param_t = Param<uint16_t>;
#endif
#ifdef ESCAPE32_U8
            static inline std::array<param_t, 33> mParams = {
#else
            static inline std::array<param_t, 40> mParams = {
#endif
                param_t{"arm", 0, 0, 1},
                param_t{"damp", 0, 0, 1},
                param_t{"revdir", 0, 0, 1},
                param_t{"brushed", 0, 0, 1},
                param_t{"timing", 0, 1, 31},
                param_t{"sine_range", 0, 0, 25},
                param_t{"sine_power", 0, 1, 15},
                param_t{"freq_min", 0, 16, 48},
                param_t{"freq_max", 0, 16, 96},
                param_t{"duty_min", 0, 1, 100},
                param_t{"duty_max", 0, 1, 100},
                param_t{"duty_spup", 0, 1, 100},
                param_t{"duty_ramp", 0, 0, 100},
                param_t{"duty_rate", 0, 1, 100},
                param_t{"duty_drag", 0, 0, 100},
                param_t{"duty_lock", 0, 0, 1},
                param_t{"throt_mode", 0, 0, 3},
                param_t{"throt_set", 0, 0, 100},
                param_t{"throt_cal", 0, 0, 1},
#ifndef ESCAPE32_U8
                param_t{"throt_min", 0, 1000, 2000},
                param_t{"throt_mid", 0, 1000, 2000},
                param_t{"throt_max", 0, 1000, 2000},
                param_t{"analog_min", 0, 0, 200},
                param_t{"analog_max", 0, 0, 3300},
#endif
                param_t{"input_mode", 0, 0, 5},
                param_t{"input_chid", 0, 0, 16},
                param_t{"telem_mode", 0, 0, 4},
                param_t{"telem_phid", 0, 0, 28},
                param_t{"telem_poles", 0, 2, 100},
#ifndef ESCAPE32_U8
                param_t{"prot_stall", 0, 0, 3500},
#endif
                param_t{"prot_temp", 0, 60, 140},
                param_t{"prot_sens", 0, 0, 2},
                param_t{"prot_volt", 0, 0, 38},
                param_t{"prot_cells", 0, 0, 24},
#ifndef ESCAPE32_U8
                param_t{"prot_curr", 0, 0, 500},
#endif
                param_t{"volume", 0, 0, 100},
                param_t{"beacon", 0, 0, 100},
                param_t{"bec", 0, 0, 3},
                param_t{"led", 0, 0, 15},
                // param_t{"unknown"}
            };

            static inline int mNextThrottle = 0;
            static inline uint8_t mNextParam = 0;

            static inline uint16_t mEscTemperature = 0;
            static inline uint16_t mMotTemperature = 0;
            static inline uint16_t mVoltage = 0;
            static inline uint16_t mCurrent = 0;
            static inline uint16_t mConsumption = 0;
            static inline uint16_t mERT = 0;
            static inline uint16_t mErrorCount = 0;
            static inline volatile etl::Event<Event> mEvent;
            static inline volatile State mState = State::Init;
            static inline External::Tick<systemTimer> mStateTick;
            static inline volatile bool mActive = false;
        };

        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct Serial {
            using clock = Config::clock;
            using systemTimer = Config::systemTimer;
            using debug = Config::debug;
            using dmaChComponent = Config::dmaChComponent;
            using pin = Config::pin;
            using tp = Config::tp;

            struct UartConfig {
                using Clock = clock;
                using ValueType = uint8_t;
                using DmaChComponent = dmaChComponent;
                static inline constexpr Mcu::Stm::Uarts::Mode mode = Mcu::Stm::Uarts::Mode::HalfDuplex;
                static inline constexpr uint32_t baudrate = RC::Protokoll::ESCape::baudrate;
                struct Rx {
                    static inline constexpr size_t size = 16;
                    static inline constexpr size_t idleMinSize = 10;
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
                static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;
                IO::outl<debug>("# Serial init");
                Mcu::Arm::Atomic::access([]{
                    mState = State::Init;
                    mErrorCount = 0;
                    mEvent = Event::None;
                    mActive = true;
                    uart::init();
                });
                pin::afunction(af);
                pin::template pullup<true>();
            }
            static inline void reset() {
                IO::outl<debug>("# Serial reset");
                Mcu::Arm::Atomic::access([]{
                    uart::reset();
                    mActive = false;
                });
                pin::analog();
            }

            enum class State : uint8_t {Init, Run};
            enum class Event : uint8_t {None, ReceiveComplete, SendComplete};

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
                        break;
                    }
                }
            }
            static inline void set(const uint16_t sbus) {
                if (mActive) {
                    if (mState == State::Run) {
                        uart::fillSendBuffer([&](auto& data){
                            const int16_t v = (sbus - RC::Protokoll::SBus::V2::mid);
                            RC::Protokoll::ESCape::CheckSum<uint8_t> cs;
                            data[0] = (cs += 0x81); // throttle + telemetry
                            data[1] = (cs += (v & 0xff));
                            data[2] = (cs += (v >> 8));
                            data[3] = cs;
                            return 4;
                        });
                        // const int16_t v = (sbus - RC::Protokoll::SBus::V2::mid);
                        // volatile uint8_t* const data = uart::outputBuffer();
                        // RC::Protokoll::ESCape::CheckSum<uint8_t> cs;
                        // data[0] = (cs += 0x81); // throttle + telemetry
                        // data[1] = (cs += (v & 0xff));
                        // data[2] = (cs += (v >> 8));
                        // data[3] = cs;
                        // uart::startSend(4);
                    }
                }
            }
            static inline void update() {
            }
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
                    if (mActive) {
                        const auto fEnable = [&]{
                            f();
                            uart::template rxEnable<true>();
                        };
                        uart::Isr::onTransferComplete(fEnable);
                    }
                }
            };
            static inline std::pair<uint8_t, uint8_t> hwVersion() {
                return {};
            }
            static inline std::pair<uint8_t, uint8_t> fwVersion() {
                return {};
            }
            static inline auto errorCount() {
                return mErrorCount;
            }
            static inline uint16_t current() {
                return mCurrent;
            }
            static inline uint16_t rpm() {
                if (mERT == std::numeric_limits<uint16_t>::max()) {
                    return 0;
                }
                return 60'000'000 / mERT;
            }

            private:
            static inline bool validityCheck(const volatile uint8_t* const /*data*/, const uint16_t size) {
                if (size != 11) {
                    return false;
                }
                return true;
            }
            static inline void readReply() {
                uart::readBuffer([&](const auto& data){
                    CheckSum<uint8_t> cs;
                    for(uint8_t i = 0; i < data.size() - 1; ++i) {
                        cs += data[i];
                    }
                    if (cs == data[data.size() - 1]) {
                        mEscTemperature = data[0];
                        mMotTemperature = data[1];
                        mVoltage = data[2] + (data[3] << 8);
                        mCurrent = data[4] + (data[5] << 8);
                        mConsumption = data[6] + (data[7] << 8);
                        mERT = data[8] + (data[9] << 8);
                    }
                    else {
                        ++mErrorCount;
                    }
                });
                // CheckSum<uint8_t> cs;
                // volatile uint8_t* const data = uart::readBuffer();
                // for(uint8_t i = 0; i < uart::readCount() - 1; ++i) {
                //     cs += data[i];
                // }
                // if (cs == data[uart::readCount() - 1]) {
                //     mEscTemperature = data[0];
                //     mMotTemperature = data[1];
                //     mVoltage = data[2] + (data[3] << 8);
                //     mCurrent = data[4] + (data[5] << 8);
                //     mConsumption = data[6] + (data[7] << 8);
                //     mERT = data[8] + (data[9] << 8);
                // }
                // else {
                //     ++mErrorCount;
                // }
            }
            static inline uint16_t mEscTemperature = 0;
            static inline uint16_t mMotTemperature = 0;
            static inline uint16_t mVoltage = 0;
            static inline uint16_t mCurrent = 0;
            static inline uint16_t mConsumption = 0;
            static inline uint16_t mERT = 0;
            static inline uint16_t mErrorCount = 0;
            static inline etl::Event<Event> mEvent;
            static inline volatile State mState = State::Init;
            static inline External::Tick<systemTimer> mStateTick;
            static inline volatile bool mActive = false;
        };
    }
}

