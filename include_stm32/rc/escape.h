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

namespace RC::ESCape {
    using namespace etl::literals;
    using namespace Units::literals;
    using namespace std::literals::chrono_literals;

    template<typename T = std::byte>
    struct CheckSum {
        T operator+=(const T b) {
            mValue = T(tbl[uint8_t(mValue ^ b)]);
            return b;
        }
        void reset() {
            mValue = T{0};
        }
        operator T() const {
            return mValue;
        }
    private:
        T mValue{};
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

    template<uint8_t N, typename Config, typename MCU = DefaultMcu>
    struct ConfigAscii {
        using clock = Config::clock;
        using systemTimer = Config::systemTimer;
        using debug = Config::debug;
        using dmaChRW = Config::dmaChRW;
        using pin = Config::pin;
        using tp = Config::tp;

        struct UartConfig {
            using Clock = clock;
            using ValueType = uint8_t;
            static inline constexpr size_t size = 1024; // send size, buffer size
            static inline constexpr size_t minSize = 2;
            using DmaChannelWrite = dmaChRW;
            using DmaChannelRead = dmaChRW;
            static inline constexpr bool useDmaTCIsr = false;
            static inline constexpr bool useIdleIsr = true;
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
            IO::outl<debug>("# ESC32Ascii init");
            __disable_irq();
            mState = State::Init;
            mErrorCount = 0;
            mEvent = Event::None;
            mActive = true;
            uart::init();
            uart::template rxEnable<false>();
            uart::baud(38'400);
            uart::template halfDuplex<true>();
            uart::template enableTCIsr<true>();
            __enable_irq();
            pin::afunction(af);
            pin::template pullup<true>();
        }
        static inline void reset() {
            IO::outl<debug>("# ESC32Ascii reset");
            __disable_irq();
            dmaChRW::enable(false);
            uart::reset();
            mActive = false;
            __enable_irq();
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
                else if (mEvent.is(Event::SendThrottle)) {
                    mState = State::SendThrot;
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
                    set(992);
                    break;
                case State::Run:
                    // IO::outl<debug>("# ESC32Ascii run");
                    break;
                case State::SendThrot:
                {
                    // IO::outl<debug>("# ESC32Ascii throt");
                    char* const data = (char*)uart::outputBuffer();
                    const uint8_t n = snprintf(data, UartConfig::size, "throt %d\n", mNextThrottle);
                    send(n);
                }
                    break;
                case State::SendParam:
                {
                    IO::outl<debug>("# ESC32Ascii param");
                    char* const data = (char*)uart::outputBuffer();
                    const uint8_t n = snprintf(data, UartConfig::size, "set %s %d\n", mParams[mNextParam].name, mParams[mNextParam].value);
                    send(n);
                }
                    break;
                case State::Save:
                {
                    IO::outl<debug>("# ESC32Ascii save");
                    char* const data = (char*)uart::outputBuffer();
                    const uint8_t n = snprintf(data, UartConfig::size, "save\n");
                    send(n);
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
            if (!mActive) return;
            if ((mState == State::Run) || (mState == State::Start)) {
                mNextThrottle = sbus2throt(sbus);
                event(Event::SendThrottle);
            }
        }
        static inline void update() {
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
            if (!mActive) return;
            if (mState == State::Run) {
                event(Event::Save);
            }
        }
        static inline void beep() {
            if (!mActive) return;
            if (mState == State::Run) {
                event(Event::Beep);
            }
        }
        static inline void setParam(const uint8_t n, const uint16_t v) {
            if (!mActive) return;
            if (mState == State::Run) {
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
        static inline void show() {
            char* const data = (char*)uart::outputBuffer();
            const uint8_t n = snprintf(data, UartConfig::size, "show\n");
            send(n);
        }
        static inline void play(const char* const music) {
            char* const data = (char*)uart::outputBuffer();
            const uint8_t n = snprintf(data, UartConfig::size, "play %s\n", music);
            send(n);
        }
        static inline int sbus2throt(const uint16_t sbus) {
            return ((sbus - 992) * 2000) / 820;
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
            const uint16_t l = uart::readCount();
            uint16_t i = 0;
            const char* p = data;
            // IO::outl<debug>("# reply: ", l);
            while(i < l) {
                if (data[i] == '\n') {
                    analyze(p, &data[i]);
                    p = &data[i + 1];
                }
                ++i;
            }
            if constexpr(!std::is_same_v<tp, void>) {
                tp::reset();
            }
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
        struct Param {
            const char* const name = nullptr;
            uint16_t value = 0;
            const uint16_t min = 0;
            const uint16_t max = 1;
        };
        static inline std::array<Param, 40> mParams = {
            Param{"arm", 0, 0, 1},
            Param{"damp", 0, 0, 1},
            Param{"revdir", 0, 0, 1},
            Param{"brushed", 0, 0, 1},
            Param{"timing", 0, 1, 31},
            Param{"sine_range", 0, 0, 25},
            Param{"sine_power", 0, 1, 15},
            Param{"freq_min", 0, 16, 48},
            Param{"freq_max", 0, 16, 96},
            Param{"duty_min", 0, 1, 100},
            Param{"duty_max", 0, 1, 100},
            Param{"duty_spup", 0, 1, 100},
            Param{"duty_ramp", 0, 0, 100},
            Param{"duty_rate", 0, 1, 100},
            Param{"duty_drag", 0, 0, 100},
            Param{"duty_lock", 0, 0, 1},
            Param{"throt_mode", 0, 0, 3},
            Param{"throt_set", 0, 0, 100},
            Param{"throt_cal", 0, 0, 1},
            Param{"throt_min", 0, 1000, 2000},
            Param{"throt_mid", 0, 1000, 2000},
            Param{"throt_max", 0, 1000, 2000},
            Param{"analog_min", 0, 0, 200},
            Param{"analog_max", 0, 0, 3300},
            Param{"input_mode", 0, 0, 5},
            Param{"input_chid", 0, 0, 16},
            Param{"telem_mode", 0, 0, 4},
            Param{"telem_phid", 0, 0, 28},
            Param{"telem_poles", 0, 2, 100},
            Param{"prot_stall", 0, 0, 3500},
            Param{"prot_temp", 0, 60, 140},
            Param{"prot_sens", 0, 0, 2},
            Param{"prot_volt", 0, 0, 38},
            Param{"prot_cells", 0, 0, 24},
            Param{"prot_curr", 0, 0, 500},
            // {"music", 0},
            Param{"volume", 0, 0, 100},
            Param{"beacon", 0, 0, 100},
            Param{"bec", 0, 0, 3},
            Param{"led", 0, 0, 15},
            Param{"unknown"}
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
        using dmaChRW = Config::dmaChRW;
        using pin = Config::pin;
        using tp = Config::tp;

        struct UartConfig {
            using Clock = clock;
            using ValueType = uint8_t;
            static inline constexpr size_t size = 11; // send size, buffer size
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
            IO::outl<debug>("# Serial init");
            __disable_irq();
            mState = State::Init;
            mErrorCount = 0;
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
            IO::outl<debug>("# Serial reset");
            __disable_irq();
            dmaChRW::enable(false);
            uart::reset();
            mActive = false;
            __enable_irq();
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
                if (const auto e = std::exchange(mEvent, Event::None); e == Event::ReceiveComplete) {
                    readReply();
                }
                else if (e == Event::SendComplete) {
                    rxEnable();
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
                    const int16_t v = (sbus - 992);
                    volatile uint8_t* const data = uart::outputBuffer();
                    CheckSum<uint8_t> cs;
                    data[0] = (cs += 0x81); // throttle + telemetry
                    data[1] = (cs += (v & 0xff));
                    data[2] = (cs += (v >> 8));
                    data[3] = cs;
                    send(4);
                }
            }
        }
        static inline void update() {
        }
        static inline void rxEnable() {
            if (mActive) {
                uart::dmaReenable([]{
                    dmaChRW::clearTransferCompleteIF();
                    dmaChRW::template setTCIsr<true>();
                    uart::dmaSetupRead2(UartConfig::minSize);
                });
                uart::template rxEnable<true>();
            }
        }
        static inline void onTransferComplete(const auto f) {
            if (mActive) {
                uart::onTransferComplete(f);
            }
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

        static inline void send(const uint8_t n) {
            uart::template rxEnable<false>();
            uart::startSend(n);
        }
        static inline void readReply() {
            CheckSum<uint8_t> cs;
            volatile uint8_t* const data = uart::readBuffer();
            for(uint8_t i = 0; i < uart::minSize - 1; ++i) {
                cs += data[i];
            }
            if (cs == data[uart::minSize - 1]) {
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
        }
        static inline uint16_t mEscTemperature = 0;
        static inline uint16_t mMotTemperature = 0;
        static inline uint16_t mVoltage = 0;
        static inline uint16_t mCurrent = 0;
        static inline uint16_t mConsumption = 0;
        static inline uint16_t mERT = 0;
        static inline uint16_t mErrorCount = 0;
        static inline Event mEvent = Event::None;
        static inline State mState = State::Init;
        static inline External::Tick<systemTimer> mStateTick;
        static inline volatile bool mActive = false;
    };

    namespace Master {
        template<uint8_t N, typename Timer, typename MCU = DefaultMcu>
        struct ProtocolAdapter {
            enum class State : uint8_t {Idle, WaitForResponseCombined, WaitForResponseSingle,
                                       Data, SingleData, Crc, CrcSingle};
            
            static inline void startWait(State s) {
                mState = s;
            }
            static inline void stopWait() {
                mState = State::Idle;
            }
            
            static inline void process(const std::byte b) {
                static CheckSum csum;
                switch(mState) {
                case State::Idle:
                break;
                case State::WaitForResponseSingle:
                    csum.reset();
                    mDataCounter = 0;
                    mData[mDataCounter++] = b; 
                    csum += b;
                    mState = State::SingleData;
                break;
                case State::SingleData:
                    mData[mDataCounter++] = b; 
                    csum += b;
                    mState = State::CrcSingle;
                break;
                case State::WaitForResponseCombined:
                    csum.reset();
                    mDataCounter = 0;
                    mData[mDataCounter++] = b; 
                    csum += b;
                    mState = State::Data;
                break;
                case State::Data:
                    mData[mDataCounter++] = b; 
                    csum += b;
                    if (mDataCounter == mData.size()) {
                        mState = State::Crc;
                    }
                break;
                case State::Crc:
                    if (csum == b) {
                        mTemperature = (uint8_t)mData[0];           
                        mVoltage = (uint8_t)mData[1] + (((uint16_t)mData[2]) << 8);           
                        mCurrent = (uint8_t)mData[3] + (((uint16_t)mData[4]) << 8);           
                        mConsumption = (uint8_t)mData[5] + (((uint16_t)mData[6]) << 8);           
                        mERT = (uint8_t)mData[7] + (((uint16_t)mData[8]) << 8);           
                    }
                    mState = State::Idle;
                break;
                case State::CrcSingle:
                    if (csum == b) {
                    }
                    mState = State::Idle;
                break;                    
                }  
            }
            
        private:
            static inline std::array<std::byte, 9> mData;
            static inline uint8_t mDataCounter;
            static inline uint8_t mTemperature;
            static inline uint16_t mVoltage;
            static inline uint16_t mCurrent;
            static inline uint16_t mConsumption;
            static inline uint16_t mERT;
            static inline State mState{State::Idle};
        };
        
        template<typename UART, typename Timer, typename MCU = DefaultMcu>
        struct Fsm {
            using uart = UART;
            using pa_t = uart::pa_t;
            
            enum class State : uint8_t {SendThrottle, SendBrake, SendLed, WaitComplete, Wait};
            static inline External::Tick<Timer> waitTicks{10ms};

            static inline void periodic() {
                switch(mState) {
                case State::WaitComplete:
                    if (uart::isIdle()) {
                        pa_t::startWait(pa_t::State::WaitForResponseCombined);
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
                switch(mState) {
                case State::SendThrottle:
                {
                    CheckSum cs;
                    uart::template rxEnable<false>();
                    cs += uart::put(0x81_B); // throttle + telemetry
                    cs += uart::put(std::byte(mThrottle & 0xff));
                    cs += uart::put(std::byte((mThrottle >> 8) & 0xff));
                    uart::put(cs);
                    mState = State::WaitComplete;
                }
                break;
                case State::SendBrake:
                {
                    CheckSum cs;
                    uart::template rxEnable<false>();
                    cs += uart::put(0x83_B); // Brake + telemetry
                    cs += uart::put(std::byte(mBrake & 0xff));
                    cs += uart::put(std::byte((mBrake >> 8) & 0xff));
                    uart::put(cs);
                    mState = State::WaitComplete;
                }
                break;
                case State::SendLed:
                {
                    CheckSum cs;
                    uart::template rxEnable<false>();
                    cs += uart::put(0x84_B); // Led + telemetry
                    cs += uart::put(std::byte(mLed & 0xff));
                    cs += uart::put(std::byte((mLed >> 8) & 0xff));
                    uart::put(cs);
                    mState = State::WaitComplete;
                }
                break;
                case State::Wait:
                    mStateTick.on(waitTicks, []{
                        pa_t::stopWait();
                        mState = State::SendThrottle; 
                    });
                break;
                default:
                break;
                }
            }
            template<auto L, auto H>
            static inline void throttle(const etl::ranged<L, H> v) {
            }
            static inline void throttle(const uint16_t sbus) {
                mThrottle = std::clamp(2000.0f * ((sbus - 992.0f) / 840.0f), -2000.0f, 2000.0f);
            }

        private:    
            static inline int16_t mThrottle{0};
            static inline uint8_t mBrake{0};
            static inline uint8_t mLed{0};
            
            static inline External::Tick<Timer> mStateTick;
            static inline State mState{State::SendThrottle};
        };
    }
}

