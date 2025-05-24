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
#include <cstring>
#include <type_traits>

#include "etl/fixedvector.h"
#include "etl/event.h"
#include "debug_pin.h"

#include "mcu/alternate.h"
#include "crc.h"
#include "tick.h"
#include "rc_2.h"
#include "crsf_2_input.h"
#include "crsf_2_output.h"
#include "usart_2.h"
#include "messagebuffer_2.h"

namespace RC {
    namespace Protokoll {
        namespace Crsf {
            namespace V4 {
                using namespace std::literals::chrono_literals;
                using namespace etl::literals;

                namespace Util {
                    template<typename PContainer, uint8_t MaxStringParamLength = 16>
                    static inline bool setParameter(PContainer& params, const uint8_t index, const auto data, const uint8_t paylength) {
                        using Param_t = typename PContainer::value_type;
                        if (index == 0) return false;
                        if (index < params.size()) {
                            bool mustSave = true;
                            if (params[index].type == Param_t::Str) {
                                // IO::outl<debug>("# String");
                                if (params[index].stringValue) {
                                    for(uint8_t i = 0; (i < MaxStringParamLength) && (i < paylength); ++i) {
                                        params[index].stringValue[i] = data[i];
                                        if (data[i] == '\0') {
                                            break;
                                        }
                                    }
                                    params[index].stringValue[paylength] = '\0';
                                }
                            }
                            else {
                                typename Param_t::value_type value{};
                                if (params[index].type <= Param_t::I8) {
                                    value = data[0];
                                    // IO::outl<debug>("# I8: v: ", value);
                                }
                                else if (params[index].type <= Param_t::I16) {
                                    value = (data[0] << 8) + data[1];
                                    // IO::outl<debug>("# I16: v: ", value);
                                }
                                else if (params[index].type <= Param_t::F32) {
                                    value = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + data[0];
                                    // IO::outl<debug>("# F32: v: ", value);
                                }
                                else if (params[index].type == Param_t::Sel) {
                                    value = data[0];
                                    // IO::outl<debug>("# Sel: v: ", value);
                                }
                                params[index].value(value);
                                if (params[index].cb) {
                                    mustSave = params[index].cb(value);
                                }
                            }
                            return mustSave;
                        }
                        return false;
                    }
                }

                template<uint8_t N, typename Config, typename MCU = DefaultMcu>
                struct Master {
                    static inline constexpr uint8_t number = N;
                    using clock = Config::clock;
                    using systemTimer = Config::systemTimer;
                    using debug = Config::debug;
                    using rxpin = Config::rxpin;
                    using txpin = Config::txpin;
                    using tp = Config::tp;
                    using callback = Config::callback;
                    using value_t = uint8_t;

                    static inline constexpr uint8_t fifoSize = Config::fifoSize;
                    static inline constexpr bool halfDuplex = std::is_same_v<rxpin, txpin>;

                    template<bool fullDuplex, typename M>
                    struct UartConfig {
                        using Clock = clock;
                        using ValueType = Master::value_t;
                        static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::FullDuplex;
                        static inline constexpr uint32_t baudrate = RC::Protokoll::Crsf::V4::baudrate;
                        struct Rx {
                            using DmaChComponent = Config::dmaChRead;
                            static inline constexpr bool enable = true;
                            static inline constexpr size_t size = RC::Protokoll::Crsf::V4::maxMessageSize;
                            static inline constexpr size_t idleMinSize = 4;
                        };
                        struct Tx {
                            using DmaChComponent = Config::dmaChWrite;
                            static inline constexpr bool singleBuffer = true;
                            static inline constexpr bool enable = true;
                            static inline constexpr size_t size = RC::Protokoll::Crsf::V4::maxMessageSize;
                        };
                        struct Isr {
                            static inline constexpr bool idle = true;
                            static inline constexpr bool txComplete = true;
                        };
                        using tp = Config::tp;
                    };
                    template<typename M>
                    struct UartConfig<false, M> {
                        using Clock = clock;
                        using ValueType = Master::value_t;
                        static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::HalfDuplex;
                        static inline constexpr uint32_t baudrate = RC::Protokoll::Crsf::V4::baudrate;
                        using DmaChComponent = Config::dmaChRW;
                        struct Rx {
                            static inline constexpr bool enable = true;
                            static inline constexpr size_t size = RC::Protokoll::Crsf::V4::maxMessageSize;
                            static inline constexpr size_t idleMinSize = 4;
                        };
                        struct Tx {
                            static inline constexpr bool singleBuffer = true;
                            static inline constexpr bool enable = true;
                            static inline constexpr size_t size = RC::Protokoll::Crsf::V4::maxMessageSize;
                        };
                        struct Isr {
                            static inline constexpr bool idle = true;
                            static inline constexpr bool txComplete = true;
                        };
                        using tp = Config::tp;
                    };
                    private:
                    using uartConfig = UartConfig<!halfDuplex, MCU>;
                    using uart = Mcu::Stm::V4::Uart<N, uartConfig, MCU>;

                    public:
                    using messageBuffer = ::Util::MessageBuffer<uart, value_t, systemTimer, tp, RC::Protokoll::Crsf::V4::maxMessageSize, fifoSize>;
                    using input  = RC::Protokoll::Crsf::V4::Input<Master>;

                    static inline constexpr uint16_t chunkBufferSize = 256;
                    using output = RC::Protokoll::Crsf::V4::Output<Master>;
                    friend output;

                    static inline void init() {
                        IO::outl<debug>("# CRSF Master init");
                        Mcu::Arm::Atomic::access([]{
                            uart::init();
                            input::init();
                            mActive = true;
                            mState = State::Init;
                            mRxEvent = Event::None;
                            mTxEvent = Event::None;
                        });
                        if constexpr(halfDuplex) {
                            static constexpr uint8_t txaf = Mcu::Stm::AlternateFunctions::mapper_v<txpin, uart, Mcu::Stm::AlternateFunctions::TX>;
                            txpin::afunction(txaf);
                            txpin::template pullup<true>();

                        }
                        else {
                            static constexpr uint8_t rxaf = Mcu::Stm::AlternateFunctions::mapper_v<rxpin, uart, Mcu::Stm::AlternateFunctions::RX>;
                            rxpin::afunction(rxaf);
                            rxpin::template pullup<true>();
                            static constexpr uint8_t txaf = Mcu::Stm::AlternateFunctions::mapper_v<txpin, uart, Mcu::Stm::AlternateFunctions::TX>;
                            txpin::afunction(txaf);
                        }
                    }
                    static inline void reset() {
                        IO::outl<debug>("# CRSF Master reset");
                        Mcu::Arm::Atomic::access([]{
                            mActive = false;
                            uart::reset();
                        });
                        rxpin::analog();
                        txpin::analog();
                    }
                    static inline void clearAll() {
                        uart::clearAll();
                    }
                    static inline uint32_t nextBaudrate() {
                        static uint8_t index = 0;
                        if (++index >= RC::Protokoll::Crsf::V4::baudrates.size()) {
                            index = 0;
                        }
                        const uint32_t br = RC::Protokoll::Crsf::V4::baudrates[index];
                        IO::outl<debug>("# nextbaud: ", br);
                        uart::baud(br);
                        return br;
                    }
                    struct Isr {
                        static inline void onTransferComplete(const auto f) {
                            if (mActive) {
                                const auto fEnable = [&]{
                                    f();
                                    mTxEvent = Event::TransmitComplete;
                                };
                                uart::Isr::onTransferComplete(fEnable);
                            }
                        }
                        static inline void onIdle(const auto f) {
                            if (mActive) {
                                const auto f2 = [&](const volatile uint8_t* const data, const uint16_t size){
                                    f();
                                    if (validityCheck(data, size)) {
                                        mRxEvent = Event::ReceiveComplete;
                                        return true;
                                    }
                                    return false;
                                };
                                uart::Isr::onIdle(f2);
                            }
                        }
                    };

                    enum class State : uint8_t {Init};
                    enum class Event : uint8_t {None, ReceiveComplete, TransmitComplete};

                    static inline void address(const std::byte adr) {
                        mAddress = (uint8_t)adr;
                    }
                    static inline void periodic() {
                        if (mRxEvent.is(Event::ReceiveComplete)) {
                            readReply();
                        }
                        else if (mTxEvent.is(Event::TransmitComplete)) {
                            messageBuffer::event(messageBuffer::Event::TransmitComplete);
                        }
                        messageBuffer::periodic();
                        output::periodic();
                    }
                    static inline void ratePeriodic() {
                        messageBuffer::ratePeriodic();
                        output::ratePeriodic();
                    }
                    private:
                    static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t) {
                        if (const uint8_t s = data[0]; ((s != (uint8_t)RC::Protokoll::Crsf::V4::Address::StartByte) &&
                                                        (s != (uint8_t)RC::Protokoll::Crsf::V4::Address::TX))) {
                            return false;
                        }
                        if (const uint8_t l = data[1]; l > RC::Protokoll::Crsf::V4::maxPayloadSize) {
                            return false;
                        }
                        if (const uint8_t t = data[2]; t > (uint8_t)RC::Protokoll::Crsf::V4::Type::Command) {
                            return false;
                        }
                        return true;
                    }
                    static inline bool crcCheck(auto data, const uint8_t paylength) {
                        CRC8 csum;
                        for(int8_t i = 0; i < paylength - 1; ++i) {
                            csum += data[i + 2];
                        }
                        if (csum == data[paylength - 1 + 2]) {
                            return true;
                        }
                        return false;
                    }
                    static inline void analyze(auto data, const uint8_t paylength) {
                        const std::byte type = (std::byte)data[2];
                        switch(type){
                        case RC::Protokoll::Crsf::V4::Type::Link:
                            input::decodeLink(data + 3);
                            if constexpr(requires(){callback::gotLinkStats();}) {
                                callback::gotLinkStats();
                            }
                            break;
                        case RC::Protokoll::Crsf::V4::Type::Channels:
                            input::decodeChannels(data + 3);
                            output::nextSlot();
                            if constexpr(requires(){callback::gotChannels();}){
                                callback::gotChannels();
                            }
                            break;
                        case RC::Protokoll::Crsf::V4::Type::Ping:
                            if constexpr(requires(){callback::disableTelemetry();}) {
                                callback::disableTelemetry();
                            }
                            if (const uint8_t dest = data[3]; ((dest == (uint8_t)Address::Broadcast) || (dest == mAddress))) {
                                const uint8_t src = data[4];
                                output::resetSlot();
                                output::setDestination((std::byte)src);
                                output::event(output::Event::SendDeviceInfo);
                                if constexpr(requires(){callback::forwardPacket(data, 0);}) {
                                    callback::forwardPacket(data, paylength + 2);
                                }
                            }
                            break;
                        case RC::Protokoll::Crsf::V4::Type::Info:
                            break;
                        case RC::Protokoll::Crsf::V4::Type::ParamEntry:
                            break;
                        case RC::Protokoll::Crsf::V4::Type::ParamRead:
                            if constexpr(requires(){callback::disableTelemetry();}) {
                                callback::disableTelemetry();
                            }
                            if (const uint8_t dest = data[3]; (dest == mAddress)) {
                                const uint8_t src = data[4];
                                const uint8_t pIndex = data[5];
                                const uint8_t pChunk = data[6];
                                output::setDestination((std::byte)src);
                                output::sendParameterInfo(pIndex, pChunk);
                            }
                            else {
                                if constexpr(requires(){callback::forwardPacket(data, 0);}) {
                                    callback::forwardPacket(data, paylength + 2);
                                }
                            }
                            break;
                        case RC::Protokoll::Crsf::V4::Type::ParamWrite:
                            if constexpr(requires(){callback::disableTelemetry();}) {
                                callback::disableTelemetry();
                            }
                            if (const uint8_t dest = data[3]; (dest == mAddress)) {
                                const uint8_t src = data[4];
                                const uint8_t pIndex = data[5];
                                const uint8_t pValue = data[6];
                                if (callback::isCommand(pIndex)) {
                                    output::setDestination((std::byte)src);
                                    output::sendCommandResponse(pIndex, pValue);
                                }
                                else {
                                    callback::setParameterValue(pIndex, data + 6, paylength - 3); // complete payload for string parameter type
                                }
                            }
                            else {
                                if constexpr(requires(){callback::forwardPacket(data, 0);}) {
                                    callback::forwardPacket(data, paylength + 2);
                                }
                            }
                            break;
                        case RC::Protokoll::Crsf::V4::Type::Command:
                            if constexpr(requires(){callback::forwardPacket(data, 0);}) {
                                callback::forwardPacket(data, paylength + 2);
                            }
                            if (const uint8_t dest = data[3]; mCommandNoAddressCheck || (dest == mAddress)) {
                                callback::command(data, paylength);
                            }
                            break;
                        }
                    }
                    static inline void readReply() {
                        uart::readBuffer([&](const auto& data){
                            uint8_t offset = 0;
                            do { // analyze all packages in one contiguous frame
                                const uint8_t paylength = std::min((uint8_t)data[1 + offset], maxPayloadSize);
                                if (!crcCheck(&data[offset], paylength)) {
                                    return;
                                }
                                analyze(&data[offset], paylength);
                                offset += paylength + 2;
                            } while(data.size() > offset);
                        });
                    }
                    inline static bool mCommandNoAddressCheck{true};
                    static inline uint8_t mAddress = 0xc8;
                    static inline volatile bool mActive = false;
                    static inline volatile etl::Event<Event> mRxEvent;
                    static inline volatile etl::Event<Event> mTxEvent;
                    static inline volatile State mState = State::Init;
                    static inline External::Tick<systemTimer> mStateTick;
                };
            }
        }
    }
}
