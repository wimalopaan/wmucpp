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
#include "etl/event.h"
#include "units.h"
#include "tick.h"
#include "rc/rc_2.h"
#include "usart_2.h"

namespace RC::Protokoll::SBus {
    namespace V2 {
		using namespace etl::literals;
		using namespace std::literals::chrono_literals;
		
		template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct Output {
			using clock = Config::clock;
            using debug = Config::debug;
            using dmaChComponent = Config::dmaChComponent;
            using systemTimer = Config::systemTimer;
            using pin = Config::pin;
            using tp = Config::tp;

            struct UartConfig {
                using Clock = clock;
                using ValueType = uint8_t;
                using DmaChComponent = dmaChComponent;
                static inline constexpr bool invert = true;
                static inline constexpr auto parity = Mcu::Stm::Uarts::Parity::Even;
                static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::TxOnly;
                static inline constexpr uint32_t baudrate = 100'000;
                struct Tx {
                    static inline constexpr bool singleBuffer = true;
                    static inline constexpr bool enable = true;
                    static inline constexpr size_t size = 26;
                };
            };

            using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;
            static inline constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;

            static inline void setChannel(const uint8_t channel, const uint16_t value) {
                output[channel] = value;
            }
            static inline void init() {
                IO::outl<debug>("# Sbus Output ", N, " init");
                for(auto& v : output) {
                    v = RC::Protokoll::SBus::V2::mid;
                }
                Mcu::Arm::Atomic::access([]{
                    uart::init();
                    mActive = true;
                });
                pin::afunction(af);
                pin::template pulldown<true>();
            }
            static inline void reset() {
                IO::outl<debug>("# Sbus2 ", N, " reset");
                Mcu::Arm::Atomic::access([]{
                    uart::reset();
                    mActive = false;
                });
                pin::analog();
            }

            static constexpr External::Tick<systemTimer> timeoutTicks{ 14ms };

            static inline void invert(const bool inv) {
				if (!mActive) return;
                if (inv) {
                    uart::template invert<true>();
                    pin::template pulldown<true>();

                } else {
                    uart::template invert<false>();
                    pin::template pullup<true>();
                }
            }
            inline static void ratePeriodic() {
				if (!mActive) return;
                ++mStateTick;
				mStateTick.on(timeoutTicks, []{
					fillSendFrame();
				});
            }
            private:
            static inline void fillSendFrame() {
                uart::fillSendBuffer([](auto& outFrame){
                    outFrame[0] = 0x0f;
                    outFrame[1] = (output[0] & 0x07FF);
                    outFrame[2] = ((output[0] & 0x07FF) >> 8 | (output[1] & 0x07FF) << 3);
                    outFrame[3] = ((output[1] & 0x07FF) >> 5 | (output[2] & 0x07FF) << 6);
                    outFrame[4] = ((output[2] & 0x07FF) >> 2);
                    outFrame[5] = ((output[2] & 0x07FF) >> 10 | (output[3] & 0x07FF) << 1);
                    outFrame[6] = ((output[3] & 0x07FF) >> 7 | (output[4] & 0x07FF) << 4);
                    outFrame[7] = ((output[4] & 0x07FF) >> 4 | (output[5] & 0x07FF) << 7);
                    outFrame[8] = ((output[5] & 0x07FF) >> 1);
                    outFrame[9] = ((output[5] & 0x07FF) >> 9 | (output[6] & 0x07FF) << 2);
                    outFrame[10] = ((output[6] & 0x07FF) >> 6 | (output[7] & 0x07FF) << 5);
                    outFrame[11] = ((output[7] & 0x07FF) >> 3);
                    outFrame[12] = ((output[8] & 0x07FF));
                    outFrame[13] = ((output[8] & 0x07FF) >> 8 | (output[9] & 0x07FF) << 3);
                    outFrame[14] = ((output[9] & 0x07FF) >> 5 | (output[10] & 0x07FF) << 6);
                    outFrame[15] = ((output[10] & 0x07FF) >> 2);
                    outFrame[16] = ((output[10] & 0x07FF) >> 10 | (output[11] & 0x07FF) << 1);
                    outFrame[17] = ((output[11] & 0x07FF) >> 7 | (output[12] & 0x07FF) << 4);
                    outFrame[18] = ((output[12] & 0x07FF) >> 4 | (output[13] & 0x07FF) << 7);
                    outFrame[19] = ((output[13] & 0x07FF) >> 1);
                    outFrame[20] = ((output[13] & 0x07FF) >> 9 | (output[14] & 0x07FF) << 2);
                    outFrame[21] = ((output[14] & 0x07FF) >> 6 | (output[15] & 0x07FF) << 5);
                    outFrame[22] = ((output[15] & 0x07FF) >> 3);
                    outFrame[23] = (mFlagsAndSwitches); // Flags byte
					outFrame[24] = 0x00;
                    return 25;
                });
            }
            static inline volatile bool mActive = false;
            static inline uint8_t mFlagsAndSwitches{};
			static inline std::array<volatile uint16_t, 16> output;
			static inline External::Tick<systemTimer> mStateTick;			
		};
		
		
		
        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct Input {
            using clock = Config::clock;
            using systemTimer = Config::systemTimer;
            using debug = Config::debug;
            using pin = Config::pin;
            using tp = Config::tp;

            static inline constexpr bool additionalChecks = []{
                if constexpr(requires(Config){Config::additionalChecks;}) {
                    return Config::additionalChecks;
                }
                return false;
            }();

            // std::integral_constant<bool, additionalChecks>::_;

            struct UartConfig {
                using debug = Config::debug;

                using pin = Config::pin; // only for software-uart
                static inline constexpr uint8_t timerN = Config::timerN;
                using callback = struct CB {
                    static inline void idle(const volatile uint8_t* const data, const uint16_t size) {
                        uart::onParityGood([&]{
                            if (validityCheck(data, size)) {
                                event(Event::ReceiveComplete);
                            }
                        });
                    }
                };

                using Clock = clock;
                using ValueType = uint8_t;
                static inline constexpr bool invert = true;
                static inline constexpr auto parity = Mcu::Stm::Uarts::Parity::Even;
                static inline constexpr bool rxtxswap = true;
                static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::RxOnly;
                static inline constexpr uint32_t baudrate = RC::Protokoll::SBus::V2::baudrate;
                using DmaChComponent = Config::dmaChComponent;
                struct Rx {
                    static inline constexpr bool enable = true;
                    static inline constexpr size_t size = 27;
                    static inline constexpr size_t idleMinSize = 4;
                };
                struct Isr {
                    static inline constexpr bool idle = true;
                };
                using tp = Config::tp;
            };

        using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;

        static inline void init() {
            IO::outl<debug>("# SBus init ", N);
            for(auto& v: mChannels) {
                v = RC::Protokoll::SBus::V2::mid;
            }
            Mcu::Arm::Atomic::access([]{
                mState = State::Init;
                mFlagsAndSwitches = 0;
                mErrorCount = 0;
                mEvent = Event::None;
                mActive = true;
                uart::init();
            });
            if constexpr(N == 0) { // software uart
                pin::template dir<Input>();
            }
            else {
                static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;
                pin::afunction(af);
            }
            pin::template pulldown<true>();
        }
        static inline void reset() {
            IO::outl<debug>("# SBus reset ", N);
            Mcu::Arm::Atomic::access([]{
                mActive = false;
                uart::reset();
            });
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
                if (mEvent.is(Event::ReceiveComplete)) {
                    readReply();
                }
                break;
            }
        }
        static inline constexpr External::Tick<systemTimer> initTicks{1000ms};
        static inline constexpr External::Tick<systemTimer> checkTicks{100ms};

        static inline void ratePeriodic() {
            if constexpr(N == 0) {
                uart::ratePeriodic();
            }
            const auto oldState = mState;
            ++mStateTick;
            switch(mState) {
            case State::Init:
                mStateTick.on(initTicks, []{
                    mState = State::Run;
                });
                break;
            case State::Run:
                mStateTick.on(checkTicks, []{
                    mPackageCountSaved = mPackageCount;
                    mPackageCount = 0;
                });
                break;
            }
            if (oldState != mState) {
                mStateTick.reset();
                switch(mState) {
                case State::Init:
                    break;
                case State::Run:
                    IO::outl<debug>("# SBus Run");
                    uart::template rxEnable<true>();
                    break;
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
                            uart::onParityGood([]{
                                event(Event::ReceiveComplete);
                                return true;
                            });
                        }
                        return false;
                    };
                    uart::Isr::onIdle(f2);
                }
            }
        };
        static inline uint16_t value(const uint8_t ch) {
            return mChannels[ch];
        }
        static inline auto errorCount() {
            return mErrorCount;
        }
        static inline auto packageCount() {
            return mPackageCountSaved;
        }
        private:
        static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t size) {
            if (size != 25) return false;
            if (data[0] != 0x0f) return false;
            if (data[24] != 0x00) return false;
            return true;
        }
        static inline void readReply() {
            ++mPackageCount;
            uart::readBuffer([](const auto& data){
                const volatile uint8_t* const mData = &data[1];

                std::array<uint16_t, 16> raw;
                raw[0]  = (uint16_t) (((mData[0]    | mData[1] << 8))                     & 0x07FF);
                raw[1]  = (uint16_t) ((mData[1]>>3  | mData[2] <<5)                     & 0x07FF);
                raw[2]  = (uint16_t) ((mData[2]>>6  | mData[3] <<2 |mData[4]<<10)  	 & 0x07FF);
                raw[3]  = (uint16_t) ((mData[4]>>1  | mData[5] <<7)                     & 0x07FF);
                raw[4]  = (uint16_t) ((mData[5]>>4  | mData[6] <<4)                     & 0x07FF);
                raw[5]  = (uint16_t) ((mData[6]>>7  | mData[7] <<1 |mData[8]<<9)   	 & 0x07FF);
                raw[6]  = (uint16_t) ((mData[8]>>2  | mData[9] <<6)                     & 0x07FF);
                raw[7]  = (uint16_t) ((mData[9]>>5  | mData[10]<<3)                     & 0x07FF);
                raw[8]  = (uint16_t) ((mData[11]    | mData[12]<<8)                     & 0x07FF);
                raw[9]  = (uint16_t) ((mData[12]>>3 | mData[13]<<5)                     & 0x07FF);
                raw[10] = (uint16_t) ((mData[13]>>6 | mData[14]<<2 |mData[15]<<10) 	 & 0x07FF);
                raw[11] = (uint16_t) ((mData[15]>>1 | mData[16]<<7)                     & 0x07FF);
                raw[12] = (uint16_t) ((mData[16]>>4 | mData[17]<<4)                     & 0x07FF);
                raw[13] = (uint16_t) ((mData[17]>>7 | mData[18]<<1 |mData[19]<<9)  	 & 0x07FF);
                raw[14] = (uint16_t) ((mData[19]>>2 | mData[20]<<6)                     & 0x07FF);
                raw[15] = (uint16_t) ((mData[20]>>5 | mData[21]<<3)                     & 0x07FF);
                bool ok = true;
                if constexpr(additionalChecks) {
                    for(uint8_t i = 0; i < raw.size(); ++i) {
                        if (!((raw[i] >= RC::Protokoll::SBus::V2::min) && (raw[i] <= RC::Protokoll::SBus::V2::max))) {
                            ok = false;
                            break;
                        }
                    }
                }
                if (ok) {
                    for(uint8_t i = 0; i < raw.size(); ++i) {
                        mChannels[i] = raw[i];
                    }
                    mFlagsAndSwitches = mData[22] & 0x0f;
                }
            });
        }
        static inline volatile bool mActive = false;
        static inline std::array<uint16_t, 16> mChannels;
        static inline uint8_t mFlagsAndSwitches = 0;
        static inline uint16_t mErrorCount = 0;
        static inline uint16_t mPackageCount = 0;
        static inline uint16_t mPackageCountSaved = 0;
        static inline volatile etl::Event<Event> mEvent;
        static inline volatile State mState = State::Init;
        static inline External::Tick<systemTimer> mStateTick;
    };
}
}
