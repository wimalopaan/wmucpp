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

#include "usart_2.h"
#include "rc/rc_2.h"
#include "rc/crsf_2.h"

using namespace std::literals::chrono_literals;

namespace RC::Protokoll::Crsf {
    namespace V4 {
        namespace Reflection {
            template<typename T>
            struct getRouter {
                using type = void;
            };
            template<typename T> requires(requires(T){typename T::router;})
            struct getRouter<T> {
                using type = T::router;
            };
            template<typename T>
            using getRouter_t = getRouter<T>::type;
        }
        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct PacketRelayRewrite {
            // N: Uart
            // src: update(): read channel from src -> Uart
            // dest: onIdle: uart-packet -> dest (buffer)

            using src = Config::src;
            using dest = Config::dest;
            using clock = Config::clock;
            using systemTimer = Config::systemTimer;
            using storage = Config::storage;
            using debug = Config::debug;
            using rxpin = Config::rxpin;
            using txpin = Config::txpin;
            using tp = Config::tp;

            using router = Reflection::getRouter_t<Config>;

            static inline constexpr uint8_t fifoSize = Config::fifoSize;

            static inline constexpr bool halfDuplex = std::is_same_v<rxpin, txpin>;

            struct UartConfig {
                struct HalfDuplex {
                    using Clock = clock;
                    using ValueType = uint8_t;
                    static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::HalfDuplex;
                    static inline constexpr uint32_t baudrate = RC::Protokoll::Crsf::V4::baudrate;
					static inline constexpr bool rxtxswap = Mcu::Stm::V4::detail::getSwap_v<Config>;
                    using DmaChComponent = Config::dmaChRead;
                    struct Rx {
                        static inline constexpr bool enable = true;
                        static inline constexpr size_t size = 64;
                        static inline constexpr size_t idleMinSize = 4;
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
                struct FullDuplex {
                    using Clock = clock;
                    using ValueType = uint8_t;
                    static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::FullDuplex;
                    static inline constexpr uint32_t baudrate = RC::Protokoll::Crsf::V4::baudrate;
                    struct Rx {
                        using DmaChComponent = Config::dmaChRead;
                        static inline constexpr bool enable = true;
                        static inline constexpr size_t size = 64;
                        static inline constexpr size_t idleMinSize = 4;
                    };
                    struct Tx {
                        using DmaChComponent = Config::dmaChWrite;
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
            };

            using uartConfig = std::conditional_t<halfDuplex, typename UartConfig::HalfDuplex, typename UartConfig::FullDuplex>;
            using uart = Mcu::Stm::V4::Uart<N, uartConfig, MCU>;

            using messageBuffer = ::Util::MessageBuffer<uart, uint8_t, systemTimer, tp, RC::Protokoll::Crsf::V4::maxMessageSize, fifoSize>;

            static inline void init() {
                IO::outl<debug>("# Relay ", N, " init");
                Mcu::Arm::Atomic::access([]{
                    uart::init();
                    mActive = true;
                    mState = State::Init;
                    mRxEvent = RxEvent::None;
                    mTxEvent = TxEvent::None;
                });
				if constexpr (Mcu::Stm::V4::detail::getSwap_v<Config>) {
					static uint8_t txaf = Mcu::Stm::AlternateFunctions::mapper_v<txpin, uart, Mcu::Stm::AlternateFunctions::RX>;
					txpin::afunction(txaf);					
				}
				else {
					static uint8_t txaf = Mcu::Stm::AlternateFunctions::mapper_v<txpin, uart, Mcu::Stm::AlternateFunctions::TX>;
					txpin::afunction(txaf);
				}
                txpin::template pullup<true>();
                if constexpr(!halfDuplex) {
                    static uint8_t rxaf = Mcu::Stm::AlternateFunctions::mapper_v<rxpin, uart, Mcu::Stm::AlternateFunctions::RX>;
                    rxpin::afunction(rxaf);
                    rxpin::template pullup<true>();
                }
            }
            static inline void reset() {
                IO::outl<debug>("# Relay ", N, " reset");
                Mcu::Arm::Atomic::access([]{
                    mActive = false;
                    uart::reset();
                });
                txpin::analog();
                if constexpr(!halfDuplex) {
                    rxpin::analog();
                }
            }
            static inline void baud(const uint32_t br) {
                if (!mActive) {
                    return;
                }
                IO::outl<debug>("# Relay Bus baud: ", br);
                uart::baud(br);
            }
            static inline void setHalfDuplex(const bool on = true)
                requires(!halfDuplex)
            {
                if (!mActive) {
                    return;
                }
                IO::outl<debug>("# Relay Bus half duplex: ", (uint8_t)on);
                uart::halfDuplex(on);
            }
            static inline void invert() {
                if (!mActive) {
                    return;
                }
                IO::outl<debug>("# Relay Bus invert");
                uart::template invert<true>();
                txpin::template pulldown<true>();
            }
            static inline void txAddress(const uint8_t adr) {
                mRewriteTxAddress = adr;
            }
            static inline void rxAddress(const uint8_t adr) {
                mRewriteRxAddress = adr;
            }

            enum class RxEvent : uint8_t {None, ReceiveComplete};
            enum class TxEvent : uint8_t {None, TransmitComplete};
            enum class State : uint8_t {Init, Run};

            static inline constexpr External::Tick<systemTimer> initTicks{100ms};
            static inline constexpr External::Tick<systemTimer> updateTicks{10ms};

            static inline void periodic() {
                using namespace RC::Protokoll::Crsf::V4;
                if (!mActive) return;
                messageBuffer::periodic();

                switch(mState) {
                case State::Init:
                    break;
                case State::Run:
                    if (mRxEvent.is(RxEvent::ReceiveComplete)) {
                        [[maybe_unused]] Debug::Scoped<tp> _tp;
                        uart::readBuffer([](const auto& data){
                            // IO::outl<debug>("# receive");
                            if (isExtendedPacket(&data.front(), data.size())) {
                                if (data[PacketIndex::type] != (uint8_t)Type::RadioID) {
                                    if constexpr(std::is_same_v<router, void>) {
                                        if (data[PacketIndex::src] == (uint8_t)Address::TX) {
                                            IO::outl<debug>("# rewrite from TX");
                                            data[PacketIndex::src] = mRewriteTxAddress;
                                            if (data[PacketIndex::type] == (uint8_t)Type::Info) {
                                                bool end = false;
                                                for(uint8_t i = 0; i < 16; ++i) {
                                                    if (data[PacketIndex::payload + i] == '\0') {
                                                        break;
                                                    }
                                                    if (!end && (i < storage::eeprom.txname.size())) {
                                                        if (storage::eeprom.txname[i] == '\0') {
                                                            data[PacketIndex::payload + i] = '.';
                                                            end = true;
                                                        }
                                                        else {
                                                            data[PacketIndex::payload + i] = storage::eeprom.txname[i];
                                                        }
                                                    }
                                                    else {
                                                        data[PacketIndex::payload + i] = '.';
                                                    }
                                                }
                                            }
                                        }
                                        else if (data[PacketIndex::src] == (uint8_t)Address::RX) {
                                            IO::outl<debug>("# rewrite from RX");
                                            data[PacketIndex::src] = mRewriteRxAddress;
                                        }
                                        if (data[PacketIndex::dest] == (uint8_t)Address::TX) {
                                            IO::outl<debug>("# rewrite To TX");
                                            data[PacketIndex::dest] = (uint8_t)Address::Handset;
                                        }
                                        IO::outl<debug>("# route");
                                        data[0] = (uint8_t)Address::StartByte;
                                        recalculateCRC(data);
                                        dest::enqueue(data);
                                    }
                                    else { // with router
                                        const uint8_t srcAddr = data[PacketIndex::src];
                                        const uint8_t rewSrcAddr = router::backwardSrcAddress(Config::id, srcAddr);
                                        IO::outl<debug>("# route: ", srcAddr, " -> ", rewSrcAddr);
                                        data[PacketIndex::src] = rewSrcAddr;
                                        data[0] = (uint8_t)Address::StartByte;
                                        recalculateCRC(data);
                                        dest::enqueue(data);
                                    }
                                }
                            }
                            else {
                                IO::outl<debug>("# telem");
                                dest::enqueue(data);
                            }
                        });
                    }
                    else if (mTxEvent.is(TxEvent::TransmitComplete)) {
                        messageBuffer::event(messageBuffer::Event::TransmitComplete);
                    }
                    break;
                }
            }
            static inline void ratePeriodic() {
                if (!mActive) return;
                messageBuffer::ratePeriodic();

                const auto oldState = mState;
                ++mStateTick;
                switch(mState) {
                case State::Init:
                    mStateTick.on(initTicks, []{
                        mState = State::Run;
                    });
                    break;
                case State::Run:
                    mStateTick.on(updateTicks, []{
                        update();
                    });
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
            struct Isr {
                static inline void onTransferComplete(const auto f) {
                    if (mActive) {
                        const auto fEnable = [&]{
                            f();
                            mTxEvent = TxEvent::TransmitComplete;
                        };
                        uart::Isr::onTransferComplete(fEnable);
                    }
                }
                static inline void onIdle(const auto f) {
                    if (mActive) {
                        const auto f2 = [&](const volatile uint8_t* const data, const uint16_t size){
                            f();
                            if (validityCheck(data, size)) {
                                mRxEvent = RxEvent::ReceiveComplete;
                                return true;
                            }
                            return false;
                        };
                        uart::Isr::onIdle(f2);
                    }
                }
            };
            static inline bool isExtendedPacket(const volatile uint8_t* const data, const uint16_t length) {
                return (length >= 6) && (data[RC::Protokoll::Crsf::V4::PacketIndex::type] >= (uint8_t)RC::Protokoll::Crsf::V4::Type::Ping);
            }
            static inline void forwardPacket(volatile uint8_t* const data, const uint16_t length) {
                using namespace RC::Protokoll::Crsf::V4;
                if (mActive) {
                    // IO::outl<debug>("# fw to TX");
                    if (isExtendedPacket(data, length)) {
                        if constexpr(std::is_same_v<router, void>) {
                            // IO::outl<debug>("# rewrite to TX");
                            if (data[PacketIndex::dest] == mRewriteTxAddress) {
                                data[PacketIndex::dest] = (uint8_t)Address::TX;
                            }
                            else if (data[PacketIndex::dest] == mRewriteRxAddress) {
                                data[PacketIndex::dest] = (uint8_t)Address::RX;
                            }
                            recalculateCRC(data);
                            messageBuffer::enqueue(std::span{data, length});
                        }
                        else { // with router
                            if ((data[PacketIndex::dest] == 0) || // broadcast
                                ((data[PacketIndex::dest] == storage::eeprom.commandBroadcastAddress) && (data[PacketIndex::type] == (uint8_t)Type::Command)))  { // pseudo-broadcast
                                IO::outl<debug>("# ping: ", data[PacketIndex::dest]);
                                messageBuffer::enqueue(std::span{data, length});
                            }
                            else {
                                const uint8_t rewriteDestAdr = router::forwardDestAddress(Config::id, data[PacketIndex::dest]);
                                if (rewriteDestAdr > 0) {
                                    data[PacketIndex::dest] = rewriteDestAdr;
                                    recalculateCRC(data);
                                    messageBuffer::enqueue(std::span{data, length});
                                }
                            }
                        }
                    }
                    else { // not extended package
						if (data[PacketIndex::type] == (uint8_t)Type::Link) {
							IO::outl<debug>("# fw lstat");							
						}
                        messageBuffer::enqueue(std::span{data, length});
                    }
                }
            }
            private:
            static inline void recalculateCRC(auto& data) {
                CRC8 crc;
                const uint8_t len = data[PacketIndex::length];
                for(uint8_t i = 0; i < len - 1; ++i) {
                    crc += data[i + PacketIndex::type];
                }
                data[len + PacketIndex::type - 1] = crc;
            }

            static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t) {
                if ((data[0] == (uint8_t)Address::StartByte) ||
                    (data[0] == (uint8_t)Address::Handset)) {
                    return true;
                }
                return false;
            }
            static inline void update() { // channels to dest
                if (mSource) {
                    messageBuffer::create_back((uint8_t)Type::Channels, [](auto& ta){
                        RC::Protokoll::Crsf::V4::pack(src::values(), ta);
                    });
                }
            }
            static inline volatile bool mSource = false;
            static inline volatile bool mActive = false;
            static inline volatile etl::Event<RxEvent> mRxEvent;
            static inline volatile etl::Event<TxEvent> mTxEvent;
            static inline volatile State mState = State::Init;
            static inline External::Tick<systemTimer> mStateTick;
            static inline uint8_t mRewriteTxAddress = 0xce;
            static inline uint8_t mRewriteRxAddress = 0xcf;
        };
    }
}
