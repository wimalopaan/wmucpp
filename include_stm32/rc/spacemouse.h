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
#include "debug_pin.h"

namespace RC::Protokoll::SpaceMouse {
    using namespace std::literals::chrono_literals;
    template<uint8_t N, typename Config, typename MCU = DefaultMcu>
    struct Input {
        using clock = Config::clock;
        using systemTimer = Config::systemTimer;
        using debug = Config::debug;
        using pin_rx = Config::pin_rx;
        using pin_tx = Config::pin_tx;
        using tp = Config::tp;

        using axis_t = std::array<uint16_t, 6>;

        inline static constexpr uint16_t halfSpan = 360;
        inline static constexpr uint16_t midValue = 8192;
        inline static constexpr uint16_t maxValue = midValue + halfSpan;
        inline static constexpr uint16_t minValue = midValue - halfSpan;
        inline static constexpr uint8_t startSymbol = 0x96;
        inline static constexpr uint8_t endSymbol = 0x8D;

        struct UartConfig {
            using Clock = clock;
            using ValueType = uint8_t;
            static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::HalfDuplex;
            static inline constexpr bool RxTxLinesDifferent = !std::is_same_v<pin_rx, pin_tx>;
            static inline constexpr uint32_t baudrate = 38'400;
            using DmaChComponent = Config::dmaChComponent;
            struct Rx {
                static inline constexpr bool enable = false;
                static inline constexpr size_t size = 16;
                static inline constexpr size_t idleMinSize = 4;
            };
            struct Tx {
                static inline constexpr bool enable = true;
                static inline constexpr bool singleBuffer = true;
                static inline constexpr size_t size = 4;
            };
            struct Isr {
                static inline constexpr bool idle = true;
                static inline constexpr bool txComplete = true;
            };
            using tp = Config::tp;
        };
        using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;
        static inline void init() {
            static constexpr uint8_t rx_af = Mcu::Stm::AlternateFunctions::mapper_v<pin_rx, uart, Mcu::Stm::AlternateFunctions::RX>;
            static constexpr uint8_t tx_af = Mcu::Stm::AlternateFunctions::mapper_v<pin_tx, uart, Mcu::Stm::AlternateFunctions::TX>;
            IO::outl<debug>("# SpMouse init ", N);
            for(auto& v: mAxis) {
                v = midValue;
            }
            Mcu::Arm::Atomic::access([]{
                mState = State::Init;
                mErrorCount = 0;
                mEvent = Event::None;
                mActive = true;
                uart::init();
            });
            pin_rx::afunction(rx_af);
            pin_rx::template pullup<true>();
            pin_tx::afunction(tx_af);
            pin_tx::template pullup<true>();
        }
        static inline void reset() {
            IO::outl<debug>("# SpMouse reset ", N);
            Mcu::Arm::Atomic::access([]{
                uart::reset();
                mActive = false;
            });
            pin_rx::analog();
            pin_tx::analog();
        }

        enum class State : uint8_t {Init, Receive, Request};
        enum class Event : uint8_t {None, ReceiveComplete};

        static inline void event(const Event e) {
            mEvent = e;
        }
        static inline void periodic() {
            const auto oldState = mState;
            switch(mState) {
            case State::Init:
                break;
            case State::Receive:
                if (mEvent.is(Event::ReceiveComplete)) {
                    readReply();
                    mState = State::Request;
                }
                break;
            case State::Request:
                break;
            }
            stateTransition(oldState);
        }
        static inline constexpr External::Tick<systemTimer> initTicks{1000ms};
        static inline constexpr External::Tick<systemTimer> waitTicks{20ms};
        static inline void ratePeriodic() {
            const auto oldState = mState;
            ++mStateTick;
            switch(mState) {
            case State::Init:
                mStateTick.on(initTicks, []{
                    mState = State::Request;
                });
                break;
            case State::Receive:
                mStateTick.on(waitTicks, []{
                    mState = State::Request;
                });
                break;
            case State::Request:
                mState = State::Receive;
                break;
            }
            stateTransition(oldState);
        }
        struct Isr {
            static inline void onTransferComplete(const auto f) {
                if (mActive) {
                    uart::Isr::onTransferComplete(f);
                }
            }
            static inline void onIdle(const auto f) {
                if (mActive) {
                    uart::Isr::onIdle([&](const volatile uint8_t* const data, const uint16_t size) {
                        f();
                        if (validityCheck(data, size)) {
                            event(Event::ReceiveComplete);
                            return true;
                        }
                        return false;
                    });
                }
            }
        };
        static inline uint16_t value(const uint8_t ch) {
            return mAxis[ch];
        }
        static inline auto errorCount() {
            return mErrorCount;
        }
        private:
        static inline void stateTransition(const State oldState) {
            if (oldState != mState) {
                mStateTick.reset();
                switch(mState) {
                case State::Init:
                    break;
                case State::Receive:
                    break;
                case State::Request:
                    requestData();
                    break;
                }
            }
        }
        static inline void requestData() {
            uart::fillSendBuffer([](auto& data){
                uint8_t n = 0;
                data[n++] = 0xac;
                return n;
            });
        }
        static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t n) {
            if (data[0] != startSymbol) {
                return false;
            }
            if (data[15] != endSymbol) {
                return false;
            }
            if (n != 16) {
                return false;
            }
            return true;
        }
        static inline void readReply() {
            uart::readBuffer([](const auto& data){
                uint16_t cs = 0;
                for(uint8_t i = 0; i < (12 + 1); ++i) {
                    cs += data[i];
                }
                uint16_t csp = (data[12 + 1] << 7) + data[13 + 1];
                if (cs == csp) {
                    ++mPackagesCount;
                    decode(data);
                }
                else {
                    ++mErrorCount;
                }
            });
        }
        static inline void decode(const auto& data) {
            for(uint8_t axis = 0; auto& v: mAxis) {
                v = (data[2 * axis + 1] << 7) + (data[2 * axis + 2]);
                ++axis;
            }
        }
        static inline volatile bool mActive = false;
        static inline axis_t mAxis;
        static inline uint16_t mErrorCount = 0;
        static inline uint16_t mPackagesCount = 0;
        static inline volatile etl::Event<Event> mEvent;
        static inline volatile State mState = State::Init;
        static inline External::Tick<systemTimer> mStateTick;
    };
}
