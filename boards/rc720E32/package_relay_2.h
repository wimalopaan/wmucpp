#pragma once

#include "usart_2.h"
#include "rc/rc_2.h"
#include "rc/crsf_2.h"

using namespace std::literals::chrono_literals;

namespace RC::Protokoll::Crsf {
    namespace V4 {
        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct PacketRelay {
            // N: Uart
            // src: update(): read channel from src -> Uart
            // dest: onIdle: uart-packet -> dest (buffer)

            using src = Config::src;
            using dest = Config::dest;
            using clock = Config::clock;
            using systemTimer = Config::systemTimer;
            using debug = Config::debug;
            using pin = Config::pin;
            using tp = Config::tp;

            struct UartConfig {
                using Clock = clock;
                using ValueType = uint8_t;
                static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::HalfDuplex;
                static inline constexpr uint32_t baudrate = RC::Protokoll::Crsf::V4::baudrate;
                using DmaChComponent = Config::dmaChComponent;
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

            using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;

            static inline void init() {
                static uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;
                IO::outl<debug>("# Relay ", N, " init");
                Mcu::Arm::Atomic::access([]{
                    uart::init();
                    mActive = true;
                    mState = State::Init;
                    mEvent = Event::None;
                });
                pin::afunction(af);
                pin::template pullup<true>();
            }
            static inline void reset() {
                IO::outl<debug>("# Relay ", N, " reset");
                Mcu::Arm::Atomic::access([]{
                    mActive = false;
                    uart::reset();
                });
                pin::analog();
            }

            enum class Event : uint8_t {None, ReceiveComplete};
            enum class State : uint8_t {Init, Run};

            static inline constexpr External::Tick<systemTimer> initTicks{100ms};

            static inline void event(const Event e) {
                mEvent = e;
            }
            static inline void periodic() {
                switch(mState) {
                case State::Init:
                    break;
                case State::Run:
                    if (mEvent.is(Event::ReceiveComplete)) {
                        tp::set();
                        uart::readBuffer([](const auto& data){
                            dest::enqueue(data);
                        });
                        tp::reset();
                    }
                    break;
                }
            }
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
            struct Isr {
                static inline void onTransferComplete(const auto f) {
                    if (mActive) {
                        const auto fEnable = [&]{
                            f();
                            uart::template rxEnable<true>();
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
            static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t) {
                if (data[0] != 0xc8) {
                    return false;
                }
                return true;
            }
            static inline void update() { // channels to dest
                uart::fillSendBuffer([](auto& data){
                    RC::Protokoll::Crsf::V4::pack(src::values(), data);
                    return 26;
                });
                // RC::Protokoll::Crsf::V4::pack(src::values(), uart::outputBuffer());
                // uart::startSend(26);
            }
#ifdef USE_EXTRA_FORWARDS
            template<typename C>
            static inline void command(const C& data, const uint16_t length) {
                uart::outputBuffer()[0] = 0xc8;
                uart::outputBuffer()[1] = length + 2;
                uart::outputBuffer()[2] = 0x32; // command
                std::copy(std::begin(data), std::begin(data) + length + 1, &uart::outputBuffer()[3]);
                uart::startSend(length + 2 + 2);
            }
            static inline void ping() {
                uart::outputBuffer()[0] = 0xc8;
                uart::outputBuffer()[1] = 0x04;
                uart::outputBuffer()[2] = 0x28; // command
                uart::outputBuffer()[3] = 0x00;
                uart::outputBuffer()[4] = 0xea;
                uart::outputBuffer()[5] = 0x54;
                uart::startSend(6);
            }
            template<typename C>
            static inline void forwardPacket(const std::byte type, const C& data, const uint16_t payLength) {
                const uint8_t crsfLength = payLength + 2;
                const uint8_t totalLength = crsfLength + 2;
                uart::outputBuffer()[0] = 0xc8;
                uart::outputBuffer()[1] = crsfLength;
                uart::outputBuffer()[2] = (uint8_t)type;
                std::copy(std::begin(data), std::begin(data) + payLength + 1, // including CRC
                          &uart::outputBuffer()[3]);
                uart::startSend(totalLength);
            }
#else
            static inline void forwardPacket(const volatile uint8_t* data, const uint16_t length) {
                uart::fillSendBuffer([&](auto& b){
                    const uint8_t l = std::min((uint16_t)b.size(), length);
                    std::copy(data, data + l, &b[0]);
                    return length;
                });
            }
#endif
            private:
            static inline volatile bool mActive = false;
            static inline volatile etl::Event<Event> mEvent;
            static inline volatile State mState = State::Init;
            static inline External::Tick<systemTimer> mStateTick;
        };
    }
}
