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
                    private:
                    using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;

                    public:
                    using messageBuffer = Util::MessageBuffer<uart, value_t, systemTimer, tp, RC::Protokoll::Crsf::V4::maxMessageSize>;
                    using input  = RC::Protokoll::Crsf::V4::Input<Master>;

                    static inline constexpr uint16_t chunkBufferSize = 256;
                    using output = RC::Protokoll::Crsf::V4::Output<Master>;
                    friend output;

                    static inline void init() {
                        static constexpr uint8_t txaf = Mcu::Stm::AlternateFunctions::mapper_v<txpin, uart, Mcu::Stm::AlternateFunctions::TX>;
                        static constexpr uint8_t rxaf = Mcu::Stm::AlternateFunctions::mapper_v<rxpin, uart, Mcu::Stm::AlternateFunctions::RX>;
                        IO::outl<debug>("# CRSF Master init");
                        Mcu::Arm::Atomic::access([]{
                            uart::init();
                            input::init();
                            mActive = true;
                            mState = State::Init;
                            mEvent = Event::None;
                        });
                        rxpin::afunction(rxaf);
                        rxpin::template pullup<true>();
                        txpin::afunction(txaf);
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
                                    event(Event::TransmitComplete);
                                };
                                uart::Isr::onTransferComplete(fEnable);
                            }
                        }
                        static inline void onIdle(const auto f) {
                            if (mActive) {
                                const auto f2 = [&](const volatile uint8_t* const data, const uint16_t size){
                                    f();
                                    if (validityCheck(data, size)) {
                                        event(Event::ReceiveComplete);
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

                    static inline void event(const Event e) {
                        mEvent = e;
                    }
                    static inline void address(const std::byte adr) {
                        mAddress = (uint8_t)adr;
                    }
                    static inline void periodic() {
                        if (mEvent.is(Event::ReceiveComplete)) {
                            readReply();
                        }
                        else if (mEvent.is(Event::TransmitComplete)) {
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
                            callback::gotLinkStats();
                            break;
                        case RC::Protokoll::Crsf::V4::Type::Channels:
                            input::decodeChannels(data + 3);
                            output::nextSlot();
                            callback::gotChannels();
                            break;
                        case RC::Protokoll::Crsf::V4::Type::Ping:
                            if (const uint8_t dest = data[3]; ((dest == (uint8_t)Address::Broadcast) || (dest == mAddress))) {
                                const uint8_t src = data[4];
                                output::resetSlot();
                                output::setDestination((std::byte)src);
                                output::event(output::Event::SendDeviceInfo);
                                callback::forwardPacket(data, paylength + 4);
                            }
                            break;
                        case RC::Protokoll::Crsf::V4::Type::Info:
                            break;
                        case RC::Protokoll::Crsf::V4::Type::ParamEntry:
                            break;
                        case RC::Protokoll::Crsf::V4::Type::ParamRead:
                            if (const uint8_t dest = data[3]; (dest == mAddress)) {
                                const uint8_t src = data[4];
                                const uint8_t pIndex = data[5];
                                const uint8_t pChunk = data[6];
                                output::setDestination((std::byte)src);
                                output::sendParameterInfo(pIndex, pChunk);
                            }
                            else {
                                callback::forwardPacket(data, paylength + 4);
                            }
                            break;
                        case RC::Protokoll::Crsf::V4::Type::ParamWrite:
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
                                callback::forwardPacket(data, paylength + 4);
                            }
                            break;
                        case RC::Protokoll::Crsf::V4::Type::Command:
                            callback::forwardPacket(data, paylength + 4);
                            if (const uint8_t dest = data[3]; mCommandNoAddressCheck || (dest == mAddress)) {
                                // const uint8_t src = data[4];
                                callback::command(data, paylength);
                            }
                            break;
                        }
                    }
                    static inline void readReply() {
                        const volatile uint8_t* const data = uart::readBuffer();
                        const uint8_t totalLength = uart::readCount();

                        uint8_t offset = 0;
                        do { // analyze all packages in one contiguous frame
                            const uint8_t paylength = std::min((uint8_t)data[1 + offset], maxPayloadSize);
                            if (!crcCheck(data + offset, paylength)) {
                                return;
                            }
                            analyze(data + offset, paylength);
                            offset += paylength + 2;
                        } while(totalLength > offset);
                    }
                    inline static bool mCommandNoAddressCheck{true};
                    static inline uint8_t mAddress = 0xc8;
                    static inline volatile bool mActive = false;
                    static inline volatile etl::Event<Event> mEvent;
                    static inline volatile State mState = State::Init;
                    static inline External::Tick<systemTimer> mStateTick;
                };
            }
        }
    }
}
