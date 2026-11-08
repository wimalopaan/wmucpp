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

#include "usart_2.h"
#include "rc/rc_2.h"
#include "rc/crsf_2.h"

using namespace std::literals::chrono_literals;

// ToDo: slotted event

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

            // optional features, enabled by the board configuration (default: off)
            template<typename T>
            consteval bool paramRetry() {
                if constexpr(requires{T::paramRetry;}) {
                    return T::paramRetry;
                }
                else {
                    return false;
                }
            }
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
            
            using tick_t = External::Tick<systemTimer, uint32_t>;
            
            using router = Reflection::getRouter_t<Config>;

			using bcastIfaces = Config::bcastInterfaces;
			
            static inline constexpr uint8_t fifoSize = Config::fifoSize;

            static inline constexpr bool halfDuplex = std::is_same_v<rxpin, txpin>;

            // Config::paramRetry: parameter read requests to devices behind the relay are
            // repeated if the answer does not arrive in time (elrsV3.lua waits 5s for every
            // lost answer from a device other than its own TX module)
            static inline constexpr bool useParamRetry = Reflection::paramRetry<Config>();

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
            static inline void setIrDA(const bool on, const bool txinvert = false)
                requires(requires{uart::template irda<true>(true, true);} && !halfDuplex)
            {
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
            }
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
                IO::outl<debug>("# act LS: ", N);
				mSendLinkStats = on;
			}
			static inline void activateChannels(const bool on) {
                IO::outl<debug>("# act CH: ", N);
				mSendRCChannels = on;
			}
			static inline void activateBroadcast(const bool on) {
                IO::outl<debug>("# act BC: ", N);
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
            static inline void disableTelemetryWithAutoOn() {
                IO::outl<debug>("# disable telemetry");
                mTelemetryTemporaryOff = true;
                mTelemTick.reset();
            }
            static inline void telemetryTempOffDuration(const uint8_t d) {
                autoOnTicks = tick_t::fromRaw(d * 1000u);                
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
                        // IO::outl<debug>("# RX E");
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
                if constexpr(useParamRetry) {
                    ++mNow;
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
                    (++mTelemTick).on(autoOnTicks, []{
                        mTelemetryTemporaryOff = false;
                        IO::outl<debug>("# telemetry enable");
                    });
                    mStateTick.on(updateTicks, []{
                        update();
                    });
                    if constexpr(useParamRetry) {
                        retryRequests();
                    }
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
                            // IO::outl<debug>("# onIdle: ", size);
                            if (validityCheck(data, size)) {
                                [[maybe_unused]] Debug::Scoped<tp> tp;
                                mRxEvent = RxEvent::ReceiveComplete;
                                return true;
                            }
                            IO::outl<debug>("# invalid relay header");
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
                if constexpr(useParamRetry) {
                    trackRequest(data, length);
                }
                messageBuffer::enqueue(std::span{data, length});
            }
            static inline void handleIncoming() {
                if constexpr(!std::is_same_v<dest, void>) {
                    uart::readBuffer([](const auto& buffer){
                        const auto handlePacket = [](auto data) {
                            const uint8_t paylength = data[PacketIndex::length];
                            if (isExtendedPacket(&data.front(), data.size())) {
                                IO::outl<debug>("# receive: ", paylength, " ", data.size());
                                if constexpr(useParamRetry) {
                                    if (!answerFromModule(data)) {
                                        return;
                                    }
                                }
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

                                    if (mTelemetryTemporaryOff) {
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
                                                    if (!Packet::createTelemetryTunnel<dest>(
                                                            data, storage::eeprom.address, PassThru::SubType::LinkStat)) {
                                                        IO::outl<debug>("# tunnel packet too large, size: ", data.size());
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    else {
                                        if (const uint8_t mode = mTelemetryForwardMode[type]; mode > 0) {
                                            if (shouldSendtelemetry(type)) {
                                                if (mode == 2) {
                                                    IO::outl<debug>("# tunnel telemtry, size: ", data.size(), " t: ", data[PacketIndex::type]);
                                                    if (!Packet::createTelemetryTunnel<dest>(
                                                            data, storage::eeprom.address, PassThru::SubType::Telemetry)) {
                                                        IO::outl<debug>("# tunnel packet too large, size: ", data.size());
                                                    }
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
                        };
                        if (!Packet::forEach(buffer, handlePacket)) {
                            IO::outl<debug>("# invalid relay packet");
                        }
                    });
                }
            }
            // ---- parameter request retry (Config::paramRetry)
            struct PendingRequest {
                uint32_t sentAt = 0;
                uint32_t lastUse = 0;
                uint32_t dupUntil = 0;
                uint32_t rtt = 0;       // smoothed round trip time (ticks)
                std::array<uint8_t, 8> req{}; // ParamRead as sent (rewritten): always 8 bytes
                uint8_t dev = 0;        // relay side address (0xEE, 0xEC, 0xC8, ...)
                uint8_t length = 0;
                uint8_t index = 0;
                uint8_t retries = 0;
                uint8_t dupIndex = 0;   // answers to be dropped (duplicates caused by our retries)
                uint8_t dupRem = 0;
                uint8_t dupCount = 0;
                bool used = false;
                bool active = false;    // waiting for an answer
            };
            static inline constexpr uint32_t msToTicks(const uint32_t ms) {
                return std::chrono::milliseconds{ms} / systemTimer::intervall;
            }
            static inline constexpr uint8_t maxRetries = 3;
            static inline PendingRequest* pendingSlot(const uint8_t dev, const bool create) {
                PendingRequest* victim = &mPending[0];
                for(auto& p : mPending) {
                    if (p.used && (p.dev == dev)) {
                        return &p;
                    }
                    if (!p.used || (victim->used && (int32_t(p.lastUse - victim->lastUse) < 0))) {
                        victim = &p;
                    }
                }
                if (!create) {
                    return nullptr;
                }
                *victim = PendingRequest{};
                victim->dev = dev;
                victim->used = true;
                return victim;
            }
            static inline uint32_t retryTimeout(const PendingRequest& p) {
                const bool local = (p.dev == (uint8_t)Address::TX); // module itself: no radio link in between
                const uint32_t rtt = (p.rtt > 0) ? p.rtt : msToTicks(local ? 20 : 300);
                const uint32_t minT = msToTicks(local ? 100 : 500);
                return std::min(msToTicks(1500), std::max(minT, 3 * rtt));
            }
            static inline void trackRequest(const volatile uint8_t* const data, const uint16_t length) {
                const uint8_t dev = data[PacketIndex::dest];
                if (dev == (uint8_t)Address::Broadcast) {
                    return;
                }
                if ((data[PacketIndex::type] == (uint8_t)Type::ParamRead) && (length >= 8) && (length <= 8)) {
                    PendingRequest& p = *pendingSlot(dev, true);
                    for(uint16_t i = 0; i < length; ++i) {
                        p.req[i] = data[i];
                    }
                    p.length = length;
                    p.index = data[PacketIndex::payload];
                    p.sentAt = mNow;
                    p.lastUse = mNow;
                    p.retries = 0;
                    p.active = true;
                }
                else if (data[PacketIndex::type] == (uint8_t)Type::ParamWrite) {
                    if (PendingRequest* p = pendingSlot(dev, false)) {
                        p->active = false; // writes are never repeated
                    }
                }
            }
            static inline void retryRequests() {
                for(auto& p : mPending) {
                    if (!p.used || !p.active || !mEnabled) {
                        continue;
                    }
                    if ((mNow - p.sentAt) < retryTimeout(p)) {
                        continue;
                    }
                    if (p.retries >= maxRetries) {
                        p.active = false;
                        continue;
                    }
                    ++p.retries;
                    p.sentAt = mNow;
                    IO::outl<debug>("# retry param read: ", p.dev, " i: ", p.index, " n: ", p.retries);
                    messageBuffer::enqueue(std::span<const volatile uint8_t>{p.req.data(), p.length});
                }
            }
            // returns false if the answer is a duplicate and must not reach the handset
            static inline bool answerFromModule(const auto& data) {
                if ((data[PacketIndex::type] != (uint8_t)Type::ParamEntry) || (data.size() < 8)) {
                    return true;
                }
                PendingRequest* const pp = pendingSlot(data[PacketIndex::src], false);
                if (!pp) {
                    return true;
                }
                PendingRequest& p = *pp;
                const uint8_t index = data[PacketIndex::payload];
                const uint8_t rem = data[PacketIndex::payload + 1];
                // exact duplicate (same entry, same chunk) of an answer already passed on
                if ((p.dupCount > 0) && (index == p.dupIndex) && (rem == p.dupRem) && (int32_t(p.dupUntil - mNow) > 0)) {
                    --p.dupCount;
                    IO::outl<debug>("# drop duplicate: ", p.dev, " i: ", index);
                    return false;
                }
                if (p.active && (index == p.index)) {
                    if (p.retries == 0) {
                        const uint32_t rtt = mNow - p.sentAt;
                        p.rtt = (p.rtt > 0) ? (p.rtt * 3 + rtt) / 4 : rtt;
                    }
                    else {
                        p.dupIndex = index;
                        p.dupRem = rem;
                        p.dupCount = p.retries;
                        p.dupUntil = mNow + msToTicks(3000);
                    }
                    p.active = false;
                }
                return true; // answer, unsolicited entry (e.g. command progress) or new
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
                        for(size_t i = 0; (i < 16) && (PacketIndex::payload + i + 15 < data.size()); ++i) {
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

            static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t size) {
                return Packet::hasValidHeader(std::span{data, size});
            }
            static inline void update() { // channels to dest
                if (mSource && mEnabled) {
                    messageBuffer::create_back((uint8_t)Type::Channels, [](auto& ta){
                        RC::Protokoll::Crsf::V4::pack(src::values(), ta);
                    });
                }
            }
            static inline tick_t autoOnTicks{30'000ms};
            
            static inline bool mTelemetryTemporaryOff  = false;
            static inline tick_t mTelemTick;
            
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

            static inline uint32_t mNow = 0; // ticks, for the parameter retry
            static inline std::array<PendingRequest, 4> mPending{};
        };
    }
}
