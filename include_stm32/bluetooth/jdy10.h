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

#include <cstddef>
#include <chrono>
#include <array>
#include <algorithm>

#include "etl/event.h"
#include "mcu/alternate.h"
#include "units.h"
#include "tick.h"
#include "rc/rc_2.h"

namespace External::Bluetooth {
    using namespace std::literals::chrono_literals;

    template<uint8_t N, typename Config, typename MCU = DefaultMcu>
    struct Simple {
        using clock = Config::clock;
        using systemTimer = Config::systemTimer;
        using debug = Config::debug;
        using pin_rx = Config::pin_rx;
        using pin_tx = Config::pin_tx;
        using tp = Config::tp;
        using swcallback = Config::swcallback;

        struct UartConfig {
            using Clock = clock;
            using ValueType = uint8_t;
            static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::HalfDuplex;
            static inline constexpr bool RxTxLinesDifferent = !std::is_same_v<pin_rx, pin_tx>;
            static inline constexpr uint32_t baudrate = 9600;
            using DmaChComponent = Config::dmaChComponent;
            struct Rx {
                static inline constexpr bool enable = true;
                static inline constexpr size_t size = 16;
                static inline constexpr size_t idleMinSize = 4;
            };
            struct Tx {
                static inline constexpr bool enable = true;
                static inline constexpr bool singleBuffer = true;
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
            static constexpr uint8_t rx_af = Mcu::Stm::AlternateFunctions::mapper_v<pin_rx, uart, Mcu::Stm::AlternateFunctions::RX>;
            static constexpr uint8_t tx_af = Mcu::Stm::AlternateFunctions::mapper_v<pin_tx, uart, Mcu::Stm::AlternateFunctions::TX>;
            IO::outl<debug>("# BT init ", N);
            Mcu::Arm::Atomic::access([]{
                mState = State::Init;
                mErrorCount = 0;
                mActive = true;
                uart::init();
            });
            pin_rx::afunction(rx_af);
            pin_rx::template pullup<true>();
            pin_tx::afunction(tx_af);
            pin_tx::template pullup<true>();
			for(auto& v : mValues) {
				v = RC::Protokoll::Crsf::V4::mid;
			}
        }
        static inline void reset() {
            IO::outl<debug>("# BT reset ", N);
            Mcu::Arm::Atomic::access([]{
                uart::reset();
                mActive = false;
            });
            pin_rx::analog();
            pin_tx::analog();
        }
        enum class Event : uint8_t {None, ReceiveComplete};
        enum class State : uint8_t {Init, Receive};

        static inline void baud(const uint32_t br) {
            if (mActive) {
                uart::baud(br);
            }
        }
        static inline void sendValue(const char* const name, const uint16_t value){
            if (mActive) {
                uart::fillSendBuffer([&](auto& data){
                    return (sprintf((char*)&data[0], "%c%s %d%c", startSymbol, name, value, endSymbol));
                });
            }
        }
        static inline void sendString(const char* const name){
            if (mActive) {
                uart::fillSendBuffer([&](auto& data){
                    return (sprintf((char*)&data[0], "%c%s%c", startSymbol, name, endSymbol));
                });
            }
        }
        static inline bool isActive() {
            return mActive;
        }
        static inline void update() {}
        static inline void event(const Event e) {
            mEvent = e;
        }
        static inline constexpr External::Tick<systemTimer> initTicks{1000ms};

        static inline void periodic() {
            mEvent.on(Event::ReceiveComplete, []{
                readReply();
            });
        }
        static inline void ratePeriodic() {
            ++mStateTick;
            switch(mState) {
            case State::Init:
                mStateTick.on(initTicks, []{
                    mState = State::Receive;
                });
                break;
            case State::Receive:
                break;
            }
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
        static inline void setLed(const uint8_t index, const bool state) {
            IO::outl<debug>("# BT set led: ", index, " ", (uint8_t)state);
            uart::fillSendBuffer([&](auto& data){
                uint8_t n = 0;
                data[n++] = '$';
                data[n++] = 'l';
                if ((index >= 10) || sendPreceedingZeros) {
                    data[n++] = '0' + ((index / 10) % 10);
                }
                data[n++] = '0' + (index % 10);
                data[n++] = ' ';
                if (state) {
                    data[n++] = '1';
                }
                else {
                    data[n++] = '0';
                }
                data[n++] = endSymbol;
                return n;
            });
        }
        static inline uint16_t value(const uint8_t ch) {
            if (ch < 16) {
                return std::clamp(mValues[ch], RC::Protokoll::SBus::V2::min, RC::Protokoll::SBus::V2::max);
            }
            else {
                return RC::Protokoll::SBus::V2::mid;
            }
        }
        static inline const auto& values() {
            return mValues;
        }
    private:
        static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t n) {
            if (n < 4) {
                return false;
            }
            if (data[0] != startSymbol) {
                return false;
            }
            if (!std::isalpha(data[1])) {
                return false;
            }
            if (!std::isdigit(data[2])) {
                return false;
            }
            return true;
        }
        static inline void readReply() {
            uart::readBuffer([](const auto& data){
                uint8_t i = 1;
                const uint8_t symbol = data[i++];
                uint8_t number = 0;
                for(; i < 5; ++i) {
                    if (std::isdigit(data[i])) {
                        number *= 10;
                        number += (data[i] - '0');
                    }
                    else {
                        break;
                    }
                }
                ++i;
                uint16_t value = 0;
                for(; i < data.size(); ++i) {
                    if (std::isdigit(data[i])) {
                        value *= 10;
                        value += (data[i] - '0');
                    }
                    else {
                        break;
                    }
                }
                IO::outl<debug>("# BT s:", symbol, " n:", number, " v:", value);

                if (std::tolower(symbol) == 'p') {
                    if (number < mValues.size()) {
                        mValues[number] = value;
                    }
                }
                else if (std::tolower(symbol) == 't') {
                    if (number < 64) {
                        const uint64_t mask = (uint64_t{1} << number);
                        mSwitches ^= mask;
                        if (mSwitches & mask) {
                            swcallback::set(number, true);
                        }
                        else {
                            swcallback::set(number, false);
                        }
						if constexpr(requires(){swcallback::state(mSwitches);}) {
							swcallback::state(mSwitches);
						}
                    }
                }
                else if (std::tolower(symbol) == 'm') {
                    if (number < 64) {
                        if (value > 0) {
                            swcallback::set(number, true);
                        }
                        else {
                            swcallback::set(number, false);
                        }
                    }
                }
            });
        }
        static inline bool sendPreceedingZeros = false;
        static inline uint64_t mSwitches = 0;
        static inline uint8_t startSymbol = '$';
        static inline uint8_t endSymbol = '\n';
        static inline std::array<uint16_t, 16> mValues{};
        static inline volatile bool mActive = false;
        static inline uint16_t mErrorCount = 0;
        static inline uint16_t mPackagesCount = 0;
        static inline etl::Event<volatile Event> mEvent;
        static inline volatile State mState = State::Init;
        static inline External::Tick<systemTimer> mStateTick;
    };

    template<uint8_t N, typename Config, typename MCU = DefaultMcu>
    struct Jdy10 {
        using clock = Config::clock;
        using systemTimer = Config::systemTimer;
        using debug = Config::debug;
        using pin_rx = Config::pin_rx;
        using pin_tx = Config::pin_tx;
        using pin_en = Config::pin_en;
        using pin_pwr = Config::pin_pwr;
        using pin_status = Config::pin_status;
        using tp = Config::tp;
        using swcallback = Config::swcallback;

        struct UartConfig {
            using Clock = clock;
            using ValueType = uint8_t;
            static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::FullDuplex;
            static inline constexpr bool RxTxLinesDifferent = !std::is_same_v<pin_rx, pin_tx>;
            static inline constexpr uint32_t baudrate = 115'200;
            struct Rx {
                using DmaChComponent = Config::dmaRxChComponent;
                static inline constexpr bool enable = true;
                static inline constexpr size_t size = 16;
                static inline constexpr size_t idleMinSize = 4;
            };
            struct Tx {
                using DmaChComponent = Config::dmaTxChComponent;
                static inline constexpr bool enable = true;
                static inline constexpr bool singleBuffer = true;
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
            static constexpr uint8_t rx_af = Mcu::Stm::AlternateFunctions::mapper_v<pin_rx, uart, Mcu::Stm::AlternateFunctions::RX>;
            static constexpr uint8_t tx_af = Mcu::Stm::AlternateFunctions::mapper_v<pin_tx, uart, Mcu::Stm::AlternateFunctions::TX>;
            IO::outl<debug>("# BT init ", N);
            Mcu::Arm::Atomic::access([]{
                mState = State::Init;
                mErrorCount = 0;
                mActive = true;
                uart::init();
            });
            pin_rx::afunction(rx_af);
            pin_rx::template pullup<true>();
            pin_tx::afunction(tx_af);
            pin_tx::template pullup<true>();

            pin_en::set();
            pin_en::template dir<Mcu::Output>();
            // pin_pwr::set(); // off
            pin_pwr::reset(); // on
            pin_pwr::template dir<Mcu::Output>();

            pin_status::template dir<Mcu::Input>();
            pin_status::template pullup<true>();
        }
        static inline void reset() {
            IO::outl<debug>("# BT reset ", N);
            Mcu::Arm::Atomic::access([]{
                uart::reset();
                mActive = false;
            });
            pin_rx::analog();
            pin_tx::analog();
        }
        enum class Event : uint8_t {None, ReceiveComplete};
        enum class State : uint8_t {Init, Receive};

        static inline void event(const Event e) {
            mEvent = e;
        }
        static inline void periodic() {
            // const auto oldState = mState;
            switch(mState) {
            case State::Init:
                break;
            case State::Receive:
                if (mEvent.is(Event::ReceiveComplete)) {
                    readReply();
                }
                break;
            }
        }
        static inline void setLed(const uint8_t index, const bool state) {
            IO::outl<debug>("# BT set led: ", index, " ", (uint8_t)state);
            uart::fillSendBuffer([&](auto& data){
                uint8_t n = 0;
                data[n++] = '$';
                data[n++] = 'l';
                if ((index >= 10) || sendPreceedingZeros) {
                    data[n++] = '0' + ((index / 10) % 10);
                }
                data[n++] = '0' + (index % 10);
                data[n++] = ' ';
                if (state) {
                    data[n++] = '1';
                }
                else {
                    data[n++] = '0';
                }
                data[n++] = endSymbol;
                return n;
            });
        }

        static inline constexpr External::Tick<systemTimer> initTicks{1000ms};
        static inline constexpr External::Tick<systemTimer> waitTicks{20ms};

        static inline void ratePeriodic() {
            // const auto oldState = mState;
            ++mStateTick;
            switch(mState) {
            case State::Init:
                mStateTick.on(initTicks, []{
                    mState = State::Receive;
                });
                break;
            case State::Receive:
                break;
            }
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
        static inline const auto& values() {
            return mValues;
        }
    private:
        static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t n) {
            if (n < 4) {
                return false;
            }
            if (data[0] != startSymbol) {
                return false;
            }
            if (!std::isalpha(data[1])) {
                return false;
            }
            if (!std::isdigit(data[2])) {
                return false;
            }
            return true;
        }
        static inline void readReply() {
            uart::readBuffer([](const auto& data){
                uint8_t i = 1;
                const uint8_t symbol = data[i++];
                uint8_t number = 0;
                for(; i < 5; ++i) {
                    if (std::isdigit(data[i])) {
                        number *= 10;
                        number += (data[i] - '0');
                    }
                    else {
                        break;
                    }
                }
                ++i;
                uint16_t value = 0;
                for(; i < data.size(); ++i) {
                    if (std::isdigit(data[i])) {
                        value *= 10;
                        value += (data[i] - '0');
                    }
                    else {
                        break;
                    }
                }
                IO::outl<debug>("# BT s:", symbol, " n:", number, " v:", value);

                if (std::tolower(symbol) == 'p') {
                    if (number < mValues.size()) {
                        mValues[number] = value;
                    }
                }
                else if (std::tolower(symbol) == 't') {
                    if (number < 64) {
                        const uint64_t mask = (uint64_t{1} << number);
                        mSwitches ^= mask;
                        if (mSwitches & mask) {
                            swcallback::set(number, true);
                        }
                        else {
                            swcallback::set(number, false);
                        }
                    }
                }
                else if (std::tolower(symbol) == 'm') {
                    if (number < 64) {
                        if (value > 0) {
                            swcallback::set(number, true);
                        }
                        else {
                            swcallback::set(number, false);
                        }
                    }
                }
            });
        }
        static inline bool sendPreceedingZeros = true;
        static inline uint64_t mSwitches = 0;
        static inline uint8_t startSymbol = '$';
        static inline uint8_t endSymbol = '\n';
        static inline std::array<uint16_t, 16> mValues{};
        static inline volatile bool mActive = false;
        static inline uint16_t mErrorCount = 0;
        static inline uint16_t mPackagesCount = 0;
        static inline volatile etl::Event<Event> mEvent;
        static inline volatile State mState = State::Init;
        static inline External::Tick<systemTimer> mStateTick;
    };
}
