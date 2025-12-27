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
            using tp1 = Config::tp1;
            using tp2 = Config::tp2;

            using router = Reflection::getRouter_t<Config>;

			using bcastIfaces = Config::bcastInterfaces;
			
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
                    using tp = Config::tp2;
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
#ifdef USE_IRDA
            static inline void setIrDA(const bool on, const bool txinvert = false) {
                if (!mActive) {
                    return;
                }
                if (on) {
                    IO::outl<debug>("# IrDA on");
                    if (txinvert) {
                        txpin::template pulldown<false>();
                        txpin::template pullup<true>();
                    }
                    else {
                        txpin::template pullup<false>();
                        txpin::template pulldown<true>();
                    }
                    rxpin::template pullup<true>();
                    rxpin::template pulldown<false>();
                }
                else {
                    IO::outl<debug>("# IrDA off");
                    txpin::template pulldown<false>();
                    txpin::template pullup<true>();
                    rxpin::template pulldown<false>();
                    rxpin::template pullup<true>();
                }
                uart::template irda<true>(on, txinvert);

                // if (on) {
                //     IO::outl<debug>("# IrDA on");
                //     txpin::template pullup<false>();
                //     txpin::template pulldown<true>();
                // }
                // else {
                //     IO::outl<debug>("# IrDA off");
                //     txpin::template pulldown<false>();
                //     txpin::template pullup<true>();
                // }
                // uart::template irda<true>(on);
            }
#endif
            static inline void enable(const bool b) {
                mEnabled = b;
            }
            static inline void activateRouter(const bool b)
                requires(!std::is_same_v<router, void>)
            {
                mUseRouter = b;
            }
            static inline void activateSource(const bool b) {
				mSource = b;
			}
            static inline void txAddress(const uint8_t adr) {
                mRewriteTxAddress = adr;
            }
            static inline void rxAddress(const uint8_t adr) {
                mRewriteRxAddress = adr;
            }
			static inline void activateLinkStats(const bool on) {
				mSendLinkStats = on;
			}
			static inline void activateChannels(const bool on) {
				mSendRCChannels = on;
			}
			static inline void activateBroadcast(const bool on) {
				mSendBCast = on;
			}
            static inline void telemetryRate(const uint8_t r) {
                mForwardRate = r;
            }
            static inline void channelsRate(const uint8_t r) {
                mChannelsRate = r;
            }
            static inline void linkStatRate(const uint8_t r) {
                mLinkStatRate = r;
            }
            enum class RxEvent : uint8_t {None, ReceiveComplete};
            enum class TxEvent : uint8_t {None, TransmitComplete};
            enum class State : uint8_t {Init, Run};

            static inline constexpr External::Tick<systemTimer> initTicks{100ms};
            static inline External::Tick<systemTimer> updateTicks{10ms};

            static inline void updateRate(const auto r) {
                updateTicks = r;
            }

			static inline void rewriteName(const bool on) {
				mDoRewriteName = on;
			}
            static inline void linkStatMode(const uint8_t m) {
                mLinkStatMode = m;
            }
            static inline void forwardTelemetryMode(const uint8_t type, const uint8_t mode) {
                if (type < mTelemetryForwardMode.size()) {
                    mTelemetryForwardMode[type] = mode;
                }
            }
            static inline void insertRoute(const uint8_t srcAddr) {
                IO::outl<debug>("# iRoute: ", Config::id, " src: ", srcAddr);
                router::backwardSrcAddress(Config::id, srcAddr, true);
            }
            static inline void periodic() {
                using namespace RC::Protokoll::Crsf::V4;
                if (!mActive) return;
                messageBuffer::periodic();

                switch(mState) {
                case State::Init:
                    break;
                case State::Run:
                    if (mRxEvent.is(RxEvent::ReceiveComplete)) {
                        IO::outl<debug>("# RX E");
                        // [[maybe_unused]] Debug::Scoped<tp> _tp;
                        handleIncoming();
                    }
                    if (mTxEvent.is(TxEvent::TransmitComplete)) {
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
                            [[maybe_unused]] Debug::Scoped<tp1> tp1;
                            f();
                            IO::outl<debug>("# onIdle: ", size);
                            if (validityCheck(data, size)) {
                                [[maybe_unused]] Debug::Scoped<tp> tp;
                                mRxEvent = RxEvent::ReceiveComplete;
                                return true;
                            }
                            // return true;
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
                if (mActive && mEnabled) {
                    //IO::outl<debug>("# fw to TX");
                    if (isExtendedPacket(data, length)) {
                        IO::outl<debug>("# ext. fw to TX");
                        if constexpr(std::is_same_v<router, void>) {
                            rewriteOutgoing(data, length);
                        }
                        else { // with router
                            if (mUseRouter) {
                                routeOutgoing(data, length);
                            }
                            else {
                                rewriteOutgoing(data, length);
                            }
                        }
                    }
                    else { // not extended package
						if (data[PacketIndex::type] == (uint8_t)Type::Link) {
							if (mSendLinkStats) {
                                if (++mLinkStatCounter >= mLinkStatRate) {
                                    messageBuffer::enqueue(std::span{data, length});
                                    mLinkStatCounter = 0;
                                }
							}
						}
						else if (data[PacketIndex::type] == (uint8_t)Type::Channels) {
							if (mSendRCChannels) {
                                if (++mChannelsCounter >= mChannelsRate) {
                                    messageBuffer::enqueue(std::span{data, length});
                                    mChannelsCounter = 0;
                                }
							}
						}
						else {
                            if (++mOtherCounter >= mOtherRate) {
                                messageBuffer::enqueue(std::span{data, length});
                                mOtherCounter = 0;
                            }
						}
                    }
                }
            }
            private:
            static inline void routeOutgoing(volatile uint8_t* const data, const uint16_t length) {
                if (((data[PacketIndex::dest] == 0) && mSendBCast) || // broadcast
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
            static inline void rewriteOutgoing(volatile uint8_t* const data, const uint16_t length) {
                if (data[PacketIndex::dest] == mRewriteTxAddress) {
                    IO::outl<debug>("# rewrite to TX, src: ", data[PacketIndex::src], " l: ", length);
                    data[PacketIndex::dest] = (uint8_t)Address::TX;
                }
                else if (data[PacketIndex::dest] == mRewriteRxAddress) {
                    IO::outl<debug>("# rewrite to RX, src: ", data[PacketIndex::src], " l: ", length);
                    data[PacketIndex::dest] = (uint8_t)Address::RX;
                }
                recalculateCRC(data);
                messageBuffer::enqueue(std::span{data, length});
            }
            static inline void handleIncoming() {
                if constexpr(!std::is_same_v<dest, void>) {
                    uart::readBuffer([](const auto& data){
                        const uint8_t paylength = std::min((uint8_t)data[1], maxPayloadSize);
                        if (!crcCheck(&data[0], paylength)) {
                            IO::outl<debug>("# wrong crc");
                            return;
                        }
                        if (isExtendedPacket(&data.front(), data.size())) {
                            IO::outl<debug>("# receive: ", paylength, " ", data.size());
                            if (data[PacketIndex::type] != (uint8_t)Type::RadioID) {
                                if constexpr(std::is_same_v<router, void>) {
                                    rewriteIncoming(data);
                                }
                                else { // with router
                                    if (mUseRouter) {
                                        // rewriteRxTxSource(data);
                                        rewriteDestination(data);
                                        if (data[PacketIndex::src] == (uint8_t)Address::TX) {
                                            rewriteName(data);
                                        }
                                        routeIncoming(data);
                                    }
                                    else {
                                        rewriteIncoming(data);
                                    }
                                }
                            }
                        }
                        else { // type < Ping
                            // IO::outl<debug>("# telem");
                            if constexpr(!std::is_same_v<dest, void>) {
                                const uint8_t type = data[PacketIndex::type];

                                if (type >= (uint8_t)Type::Ping) {
                                    IO::outl<debug>("# wrong type");
                                    return;
                                }

                                if (type == (uint8_t)Type::Link) {
                                    if (mLinkStatMode > 0) {
                                        if (shouldSendtelemetry(type)) {
                                            if (mLinkStatMode == 1) { // forward
                                                IO::outl<debug>("# return link stats");
                                                dest::enqueue(data);
                                            }
                                            else if (mLinkStatMode == 2) { // transform
                                                const uint8_t up_rssi1 = data[PacketIndex::type + 1 + 0];
                                                const uint8_t up_rssi2 = data[PacketIndex::type + 1 + 1];
                                                const uint8_t up_link_qly  = data[PacketIndex::type + 1 + 2];
                                                const uint8_t up_snr  = data[PacketIndex::type + 1 + 3];
                                                // const uint8_t active_ant  = data[PacketIndex::type + 1 + 4];
                                                const uint8_t rf_profile  = data[PacketIndex::type + 1 + 5];
                                                const uint8_t up_rf_power  = data[PacketIndex::type + 1 + 6]; // not correctly displayed in EdgeTx (missing lookup)
                                                const uint8_t dn_rssi  = data[PacketIndex::type + 1 + 7];
                                                const uint8_t dn_link_qly  = data[PacketIndex::type + 1 + 8];
                                                const uint8_t dn_snr  = data[PacketIndex::type + 1 + 9];
                                                IO::outl<debug>("# link rx tx: ", data.size());
                                                dest::create_back((uint8_t)Type::LinkStatsRx, [&](auto& ta){
                                                    ta.push_back(dn_rssi);
                                                    ta.push_back(dn_link_qly);
                                                    ta.push_back(dn_link_qly); // ???
                                                    ta.push_back(dn_snr);
                                                    ta.push_back(up_rf_power);
                                                });
                                                dest::create_back((uint8_t)Type::LinkStatsTx, [&](auto& ta){
                                                    ta.push_back(std::min(up_rssi1, up_rssi2)); // makes sense?
                                                    ta.push_back(up_link_qly);
                                                    ta.push_back(up_link_qly); // ???
                                                    ta.push_back(up_snr);
                                                    ta.push_back(uint8_t(0)); // ???
                                                    ta.push_back(rf_profile); // ???
                                                });
                                            }
                                            else if (mLinkStatMode == 3) { // tunnel
                                                IO::outl<debug>("# tunnel link stats, size: ", data.size());
                                                dest::create_back((uint8_t)Type::PassThru, [&](auto& ta){
                                                    ta.push_back(Address::Handset);
                                                    ta.push_back(storage::eeprom.address);
                                                    ta.push_back(PassThru::AppId::Telem);
                                                    ta.push_back(PassThru::SubType::LinkStat);
                                                    for(uint8_t i = PacketIndex::type; i < (data.size() - 1); ++i) {
                                                        ta.push_back(data[i]);
                                                    }
                                                });
                                            }
                                        }
                                    }
                                }
                                else {
                                    if (const uint8_t mode = mTelemetryForwardMode[type]; mode > 0) {
                                        if (shouldSendtelemetry(type)) {
                                            if (mode == 2) {
                                                IO::outl<debug>("# tunnel telemtry, size: ", data.size(), " t: ", data[PacketIndex::type]);
                                                dest::create_back((uint8_t)Type::PassThru, [&](auto& ta){
                                                    ta.push_back(Address::Handset);
                                                    ta.push_back(storage::eeprom.address);
                                                    ta.push_back(PassThru::AppId::Telem);
                                                    ta.push_back(PassThru::SubType::Telemetry);
                                                    for(uint8_t i = PacketIndex::type; i < (data.size() - 1); ++i) {
                                                        ta.push_back(data[i]);
                                                    }
                                                });
                                            }
                                            else if (mode == 1) {
                                                IO::outl<debug>("# telemtry, size: ", data.size());
                                                dest::enqueue(data);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    });
                }
            }
            static inline bool shouldSendtelemetry(const uint8_t type) {
                if (type < mTypeCounter.size()) {
                    if (++mTypeCounter[type] >= mForwardRate) {
                        mTypeCounter[type] = 0;
                        return true;
                    }
                }
                return false;
            }
            static inline void routeIncoming(auto& data) {
                const uint8_t srcAddr = data[PacketIndex::src];
                const uint8_t rewSrcAddr = router::backwardSrcAddress(Config::id, srcAddr);
                IO::outl<debug>("# route: ", srcAddr, " -> ", rewSrcAddr);
                data[PacketIndex::src] = rewSrcAddr;
                data[0] = (uint8_t)Address::StartByte;
                recalculateCRC(data);
                dest::enqueue(data);

                // better: send only to ifaces where the dest address is
                Meta::visit<bcastIfaces>([&]<typename IF>(Meta::Wrapper<IF>){
                    IF::forwardPacket(&data[0], data.size());
                });
            }
            static inline void rewriteDestination(auto& data) {
                if (data[PacketIndex::dest] == (uint8_t)Address::TX) {
                    IO::outl<debug>("# rewrite to Handset, dest: ", data[PacketIndex::dest]);
                    data[PacketIndex::dest] = (uint8_t)Address::Handset;
                }
            }
            static inline void rewriteName(auto& data) {
                if constexpr(requires(){storage::eeprom.txname[0];}) {
                    if (mDoRewriteName && (data[PacketIndex::type] == (uint8_t)Type::Info)) {
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
            }
            static inline void rewriteRxTxSource(auto& data) {
                if (data[PacketIndex::src] == (uint8_t)Address::TX) {
                    IO::outl<debug>("# rewrite from TX");
                    data[PacketIndex::src] = mRewriteTxAddress;
                    rewriteName(data);
                }
                else if (data[PacketIndex::src] == (uint8_t)Address::RX) {
                    IO::outl<debug>("# rewrite from RX, dest: ", data[PacketIndex::dest]);
                    data[PacketIndex::src] = mRewriteRxAddress;
                }
            }
            static inline void rewriteIncoming(auto& data) {
                rewriteRxTxSource(data);
                rewriteDestination(data);
                if constexpr(std::is_same_v<router, void>) {
                    IO::outl<debug>("# rew in");
                }
                else {
                    IO::outl<debug>("# rew id: ", Config::id);
                }
                data[0] = (uint8_t)Address::StartByte;
                recalculateCRC(data);
                if constexpr(!std::is_same_v<dest, void>) {
                    dest::enqueue(data);
                }
            }
            static inline void recalculateCRC(auto& data) {
                CRC8 crc;
                const uint8_t len = data[PacketIndex::length];
                for(uint8_t i = 0; i < len - 1; ++i) {
                    crc += data[i + PacketIndex::type];
                }
                data[len + PacketIndex::type - 1] = crc;
            }

            static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t) {
                if (const uint8_t s = data[0]; ((s != (uint8_t)RC::Protokoll::Crsf::V4::Address::StartByte) &&
                                                (s != (uint8_t)RC::Protokoll::Crsf::V4::Address::Handset))) {
                    return false;
                }
                if (const uint8_t l = data[1]; l > RC::Protokoll::Crsf::V4::maxPayloadSize) {
                    return false;
                }
                else if (l < RC::Protokoll::Crsf::V4::minPayloadSize) {
                    return false;
                }
                if (const uint8_t t = data[2]; t > (uint8_t)RC::Protokoll::Crsf::V4::Type::Last) {
                    return false;
                }
                return true;
            }
            static inline bool crcCheck(const auto& data, const uint8_t paylength) {
                CRC8 csum;
                for(int8_t i = 0; i < paylength - 1; ++i) {
                    csum += data[i + 2];
                }
                if (csum == data[paylength - 1 + 2]) {
                    return true;
                }
                return false;
            }
            static inline void update() { // channels to dest
                if (mSource && mEnabled) {
                    messageBuffer::create_back((uint8_t)Type::Channels, [](auto& ta){
                        RC::Protokoll::Crsf::V4::pack(src::values(), ta);
                    });
                }
            }
            static inline bool mEnabled  = true;
            static inline bool mSendBCast  = true;
            static inline bool mSendLinkStats  = true;
            static inline bool mSendRCChannels = true;
            static inline bool mSource = false;
            static inline bool mUseRouter = true;
            static inline volatile bool mActive = false;
            static inline volatile etl::Event<RxEvent> mRxEvent;
            static inline volatile etl::Event<TxEvent> mTxEvent;
            static inline volatile State mState = State::Init;
            static inline External::Tick<systemTimer> mStateTick;
            static inline uint8_t mRewriteTxAddress = 0xce;
            static inline uint8_t mRewriteRxAddress = 0xcf;
			static inline bool mDoRewriteName = false; 
            static inline uint8_t mLinkStatMode = 1; // 0=Off; 1=Forward; 2=Transform; 3=Tunnel
            static inline std::array<uint8_t, (uint8_t)Type::Ping> mTelemetryForwardMode{}; // 0=Off; 1=Forward; 2=Tunnel
            static inline uint8_t mForwardRate = 2;
            static inline std::array<uint8_t, (uint8_t)Type::Ping> mTypeCounter{};
            static inline uint8_t mChannelsRate = 1;
            static inline uint8_t mChannelsCounter = 0;
            static inline uint8_t mLinkStatRate = 1;
            static inline uint8_t mLinkStatCounter = 0;
            static inline uint8_t mOtherRate = 1;
            static inline uint8_t mOtherCounter = 0;
        };
    }
}
