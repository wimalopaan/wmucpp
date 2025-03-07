/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "mcu/alternate.h"
#include "byte.h"
#include "etl/algorithm.h"
#include "etl/ranged.h"
#include "units.h"
#include "tick.h"
#include "rc/rc.h"

template<uint8_t N, typename Config, typename MCU = DefaultMcu>
struct IBusInput {
    using clock = Config::clock;
    using systemTimer = Config::systemTimer;
    using debug = Config::debug;
    using dmaChRead = Config::dmaChRead;
    using pin = Config::pin;
    using tp = Config::tp;

    struct UartConfig {
        using Clock = clock;
        using ValueType = uint8_t;
        static inline constexpr size_t size = 34; // buffer size 32 + X
        static inline constexpr size_t minSize = 4; // receive size
        using DmaChannelWrite = void;
        using DmaChannelRead = dmaChRead;
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
        IO::outl<debug>("# IBus init");
        __disable_irq();
        mState = State::Init;
        mErrorCount = 0;
        mEvent = Event::None;
        mActive = true;
        uart::init();
        uart::template txEnable<false>();
        uart::baud(115'200);
        uart::template halfDuplex<true>();
        dmaChRead::enable(false);
        __enable_irq();
        for(auto& v: mChannels) {
            v = 992;
        }
        pin::afunction(af);
        pin::template pullup<true>();
    }
    static inline void reset() {
        IO::outl<debug>("# IBus reset");
        __disable_irq();
        mActive = false;
        dmaChRead::enable(false);
        uart::reset();
        __enable_irq();
        pin::analog();
    }

    enum class State : uint8_t {Init, Run};
    enum class Event : uint8_t {None, ReceiveComplete};

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
    static inline void update() {
    }
    static inline void onIdle(const auto f) {
        if (mActive) {
            uart::onIdle([&]{
                f();
                uart::rxClear();
                dmaChRead::count(UartConfig::size);
            });
        }
    }
    static inline uint16_t value(const uint8_t ch) {
        return mChannels[ch];
    }
    static inline auto errorCount() {
        return mErrorCount;
    }
    private:
    static inline constexpr uint16_t ibmax = 2011;
    static inline constexpr uint16_t ibmin = 988;
    static inline constexpr uint16_t sbmax = 1812;
    static inline constexpr uint16_t sbmin = 172;

    static inline uint16_t ibus2sbus(const uint16_t ib) {
        const int ic = std::clamp(ib, ibmin, ibmax);
        const int sb = ((uint32_t)(ic - ibmin) * (sbmax - sbmin)) / (ibmax - ibmin) + sbmin;
        return std::clamp(sb, 172, 1812);
    }

    struct CheckSum final {
        inline uint8_t operator+=(const uint8_t b) {
            mSum -= b;
            return b;
        }
        inline uint8_t highByte() const {
            return mSum >> 8;
        }
        inline uint8_t lowByte() const {
            return mSum;
        }
        inline void highByte(const uint8_t hb) {
            mH = hb;
        }
        inline void lowByte(const uint8_t lb){
            mL = lb;
        }
        inline explicit operator bool() const {
            return (lowByte() == mL) && (highByte() == mH);
        }
    private:
        uint8_t mH{};
        uint8_t mL{};
        uint16_t mSum = std::numeric_limits<uint16_t>::max();
    };

    static inline void decode(const uint8_t ch) {
        const volatile uint8_t* const data = uart::readBuffer();
        if (ch < 14) {
            const uint8_t h = data[ch * 2 + 1 + 2] & 0x0f;
            const uint8_t l = data[ch * 2 + 2];
            const uint16_t  v = (uint16_t(h) << 8) + uint8_t(l);
            mChannels[ch] = ibus2sbus(v);
        }
        else if (ch < 18) {
            const uint8_t h1 = data[6 * (ch - 14) + 1 + 2] & 0xf0;
            const uint8_t h2 = data[6 * (ch - 14) + 3 + 2] & 0xf0;
            const uint8_t h3 = data[6 * (ch - 14) + 5 + 2] & 0xf0;
            const uint16_t v = (uint8_t(h1) >> 4) + uint8_t(h2) + (uint16_t(h3) << 4);
            mChannels[ch] = ibus2sbus(v);
        }
    }

    static inline void readReply() {
        const volatile uint8_t* const data = uart::readBuffer();
        uint8_t i = 0;
        CheckSum cs;

        if ((cs += data[i++]) != 0x20) return;
        if ((cs += data[i++]) != 0x40) return;

        for(uint8_t k = 0; k < 28; ++k) {
            cs += data[i++];
        }
        cs.lowByte(data[i++]);
        cs.highByte(data[i++]);
        if (cs) {
            for(uint8_t k = 0; k < 18; ++k) {
                decode(k);
            }
        }
        else {
            ++mErrorCount;
        }
    }
    static inline volatile bool mActive = false;
    static inline std::array<uint16_t, 18> mChannels; // sbus
    static inline uint16_t mErrorCount = 0;
    static inline volatile Event mEvent = Event::None;
    static inline volatile State mState = State::Init;
    static inline External::Tick<systemTimer> mStateTick;
};

