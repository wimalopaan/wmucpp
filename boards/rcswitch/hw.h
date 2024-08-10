#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <chrono>

#include <etl/ranged.h>
#include <etl/algorithm.h>

#include <external/solutions/tick.h>

#include <mcu/internals/usart.h>

namespace External {
    namespace HW {

        static inline constexpr std::byte start_byte = 0xaa_B;

        namespace Output {
            using namespace std::literals::chrono;

            template<typename CN, typename Timer, uint8_t CNumber = 0x00, typename dbg = void, uint8_t Size = 16>
            struct Generator final {
                static inline constexpr External::Tick<Timer> timeoutTicks{100_ms};
                static_assert(timeoutTicks.value > 1);

                template<auto N>
                struct PAdapter final {
                    enum class State : uint8_t {Idle, GotStart, GotController, GotType, GotLength, GotPayLoad, Send, Wait};

                    static inline void ratePeriodic() requires(CNumber > 0){
                        if (mState == State::Idle) {
                            rxEnable<true>();
                        }
                        else if (mState == State::Send) {
                            putNextPacket();
                            mState = State::Wait;
                        }
                    }
                    static inline void periodic() requires(CNumber > 0){
                        if (mState == State::Wait) {
                            if (isIdle()) {
                                mState = State::Idle;
                            }
                        }
                    }

                    static inline bool process(const std::byte b) requires(CNumber > 0){
                        static uint8_t length = 0;
                        static uint8_t csum = 0;
                        static bool match = false;
                        switch(mState) {
                        case State::Idle:
                            if (b == 0xaa_B) {
                                length = 0;
                                mState = State::GotStart;
                            }
                            break;
                        case State::GotStart:
                            if (uint8_t(b) <= 7) {
                                const uint8_t prevNumber = uint8_t(b);
                                match = ((CNumber - 1) == prevNumber);
                                mState = State::GotController;
                            }
                            else {
                                mState = State::Idle;
                            }
                            break;
                        case State::GotController:
                            if (uint8_t(b) <= 1) {
                                mState = State::GotType;
                            }
                            else {
                                mState = State::Idle;
                            }
                            break;
                        case State::GotType:
                            if (uint8_t(b) <= 8) {
                                length = uint8_t(b);
                                mState = State::GotLength;
                            }
                            else {
                                mState = State::Idle;
                            }
                            break;
                        case State::GotLength:
                            if (length > 0) {
                                --length;
                                csum += uint8_t(b);
                                if (length == 0) {
                                    mState = State::GotPayLoad;
                                }
                            }
                            else {
                                mState = State::GotPayLoad;
                            }
                            break;
                        case State::GotPayLoad:
                            if (csum == uint8_t(b)) {
                                if (match) {
                                    ++packagesCounter;
                                    failCounter = 10;
                                    mState = State::Send;
                                    rxEnable<false>();
                                }
                                else {
                                    mState = State::Idle;
                                }
                            }
                            else {
                                mState = State::Idle;
                            }
                            break;
                        case State::Send:
                            break;
                        case State::Wait:
                            break;
                        }
                        return true;
                    }
                    private:
                    static inline State mState{State::Idle};
                };

                using pa = std::conditional_t<CNumber == 0x00, External::Hal::NullProtocollAdapter<>, PAdapter<0>>;
                using rql = std::conditional_t<CNumber == 0x00, AVR::ReceiveQueueLength<2>, AVR::ReceiveQueueLength<0>>;
                using usart = AVR::Usart<CN, pa, AVR::UseInterrupts<false>, rql, AVR::SendQueueLength<Size>>;

                inline static void init() {
                    usart::template init<AVR::BaudRate<115200>, AVR::HalfDuplex>();
                    usart::txOpenDrain();
                    usart::template txPinPullup<true>();
                    usart::template rxEnable<false>();
                }

                static inline void set(const uint8_t v) {
                    data = std::byte(v);
                }

                inline static void periodic() {
                    usart::periodic();
                    if constexpr(CNumber > 0) {
                        pa::periodic();
                    }
                }

                inline static void ratePeriodic() {
                    if constexpr(CNumber == 0x00) {
                        (++ticks).on(timeoutTicks, []{
                            if constexpr(!std::is_same_v<dbg, void>) {
                                dbg::toggle();
                            }
                            putNextPacket();
                        });
                    }
                    else {
                        pa::ratePeriodic();
                        (++ticks).on(timeoutTicks, []{
                            if (failCounter > 0) {
                                --failCounter;
                            }
                        });
                    }
                }
                static inline bool fail() {
                    return failCounter == 0;
                }
            private:
                inline static bool isIdle() {
                    return usart::isIdle();
                }
                template<bool B>
                inline static void rxEnable() {
                    usart::template rxEnable<B>();
                }
                inline static void putNextPacket() {
                    usart::put(start_byte);
                    usart::put(std::byte(CNumber)); // controller number
                    usart::put(0x00_B); // type 0: binary switches
                    usart::put(0x01_B); // payload length (1 byte)
                    usart::put(data); // payload
                    usart::put(data); // crc = sum of payload
                }
                static inline uint8_t failCounter{10};
                static inline uint16_t packagesCounter{};
                static inline std::byte data{};
                static inline External::Tick<Timer> ticks{};
            };
        }
    }
}
