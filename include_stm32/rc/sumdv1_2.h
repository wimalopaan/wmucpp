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

#include "mcu/alternate.h"
#include "byte.h"
#include "etl/algorithm.h"
#include "etl/ranged.h"
#include "units.h"
#include "tick.h"
#include "rc/rc_2.h"

namespace RC::Protokoll::SumDV1 {
namespace V2 {
using namespace etl::literals;
using namespace std::literals::chrono_literals;

template<uint8_t N, typename Config, typename MCU = DefaultMcu>
struct Output {
    using clock = Config::clock;
    using systemTimer = Config::systemTimer;
    using debug = Config::debug;
    using pin = Config::pin;
    using tp = Config::tp;

    struct UartConfig {
        using Clock = clock;
        using ValueType = uint8_t;
        using Adapter = void;
        static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::TxOnly;
        static inline constexpr uint32_t baudrate = RC::Protokoll::Hott::SumDV3::V2::baudrate;
        using DmaChComponent = Config::dmaChComponent;
        struct Tx {
            static inline constexpr bool singleBuffer = true;
            static inline constexpr bool enable = true;
            static inline constexpr size_t size = 64;
        };
        using tp = Config::tp;
    };
    using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;
    static inline constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;

    static inline void init() {
        IO::outl<debug>("# SumDV1Out init");
        for(auto& v : mChannels) {
            v = RC::Protokoll::Hott::SumDV3::V2::CenterValue;
        }
        Mcu::Arm::Atomic::access([]{
            uart::init();
            mState = State::Init;
            mActive = true;
        });
        pin::afunction(af);
    }
    static inline void reset() {
        IO::outl<debug>("# SumDV1Out reset");
        Mcu::Arm::Atomic::access([]{
            mActive = false;
            uart::reset();
        });
        pin::analog();
    }

    enum class State : uint8_t {Init, Run};

    static inline void set(const uint8_t channel, const uint16_t value) {
        if (channel < mChannels.size()) {
            const int32_t d = (value - RC::Protokoll::Crsf::V4::mid);
            const int32_t scaled = (d * RC::Protokoll::Hott::SumDV3::V2::Span) / RC::Protokoll::Crsf::V4::span;
            mChannels[channel] = scaled + RC::Protokoll::Hott::SumDV3::V2::CenterValue;
        }
    }
    static inline void update() {
    }
    static inline void periodic() {
    }

    static inline constexpr External::Tick<systemTimer> initTicks{100ms};
    static inline constexpr External::Tick<systemTimer> nextTicks{14ms};

    static inline void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::Init:
            mStateTick.on(initTicks, []{
                mState = State::CH1_16;
            });
            break;
        case State::Run:
            mStateTick.on(nextTicks, []{
                sendData();
            });
            break;
        }
        if (oldState != mState) {
            switch(mState) {
            case State::Init:
                break;
            case State::Run:
                break;
            }
        }
    }
private:
    static inline void sendData() {
        uart::fillSendBuffer([&](auto& data){
            uint8_t i = 0;
            RC::Protokoll::Hott::SumDV3::V2::Crc16 crc;
            crc += etl::assign(data[i++], RC::Protokoll::Hott::SumDV3::V2::start_code);
            crc += etl::assign(data[i++], RC::Protokoll::Hott::SumDV3::V2::version_code1);
            crc += etl::assign(data[i++], 16);

            for(uint8_t c = 0; c < 16; c++) {
                crc += etl::assignH(data[i++], mChannels[c]);
                crc += etl::assignL(data[i++], mChannels[c]);
            }

            data[i++] = crc >> 8;
            data[i++] = crc & 0xff;
            return i;
        });
    }
    static inline volatile bool mActive = false;
    static inline volatile State mState = State::Init;
    static inline External::Tick<systemTimer> mStateTick;
    static inline std::array<uint16_t, 16> mChannels{};
};

}
}


