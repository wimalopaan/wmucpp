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
#include "rc/rc_2.h"
#include "debug_pin.h"

namespace RC::Protokoll::IBus {
    namespace V2 {
        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct Input {
            using clock = Config::clock;
            using systemTimer = Config::systemTimer;
            using debug = Config::debug;
            using dmaChComponent = Config::dmaChComponent;
            using pin = Config::pin;
            using tp = Config::tp;

            struct UartConfig {
                using Clock = clock;
                using ValueType = uint8_t;
                using Adapter = void;
                static inline constexpr bool rxtxswap = true;
                static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::RxOnly;
                static inline constexpr uint32_t baudrate = RC::Protokoll::IBus::V2::baudrate;
                using DmaChComponent = Config::dmaChComponent;
                struct Rx {
                    static inline constexpr bool enable = true;
                    static inline constexpr size_t size = 34;
                    static inline constexpr size_t idleMinSize = 4;
                };
                struct Isr {
                    static inline constexpr bool idle = true;
                };
                using tp = Config::tp;
            };
            using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;
            static inline void init() {
                static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;
                IO::outl<debug>("# IBus init");
                for(auto& v: mChannels) {
                    v = RC::Protokoll::SBus::V2::mid;
                }
                Mcu::Arm::Atomic::access([]{
                    mState = State::Init;
                    mErrorCount = 0;
                    mEvent = Event::None;
                    mActive = true;
                    uart::init();
                });
                pin::afunction(af);
                pin::template pullup<true>();
            }
            static inline void reset() {
                IO::outl<debug>("# IBus reset");
                Mcu::Arm::Atomic::access([]{
                    uart::reset();
                    mActive = false;
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
                        IO::outl<debug>("# IBus Run");
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
                                event(Event::ReceiveComplete);
                                return true;
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
            private:
            static inline uint16_t ibus2sbus(const uint16_t ib) {
                const int ic = std::clamp(ib, RC::Protokoll::IBus::V2::min, RC::Protokoll::IBus::V2::max);
                const int sb = ((uint32_t)(ic - RC::Protokoll::IBus::V2::min) * RC::Protokoll::SBus::V2::amp) / RC::Protokoll::IBus::V2::amp + RC::Protokoll::SBus::V2::min;
                return std::clamp(sb, RC::Protokoll::SBus::V2::min, RC::Protokoll::SBus::V2::max);
            }
            static inline void decode(const uint8_t ch) {
                const volatile uint8_t* const data = uart::readBuffer();
                if (ch < 14) {
                    const uint8_t h = data[ch * 2 + 1 + 2] & 0x0f;
                    const uint8_t l = data[ch * 2 + 2];
                    const uint16_t  v = (uint16_t(h) << 8) + uint8_t(l);
                    mChannels[ch] = ibus2sbus(v);
                }
                else if (ch < 18) {
                    const uint8_t h1 = data[6 * (ch - 14) + 1 + 2] & 0xf0;
                    const uint8_t h2 = data[6 * (ch - 14) + 3 + 2] & 0xf0;
                    const uint8_t h3 = data[6 * (ch - 14) + 5 + 2] & 0xf0;
                    const uint16_t v = (uint8_t(h1) >> 4) + uint8_t(h2) + (uint16_t(h3) << 4);
                    mChannels[ch] = ibus2sbus(v);
                }
            }
            static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t) {
                if (data[0] != 0x20) {
                    return false;
                }
                if (data[1] != 0x40) {
                    return false;
                }
                return true;
            }
            static inline void readReply() {
                const volatile uint8_t* const data = uart::readBuffer();
                uint8_t i = 0;
                CheckSum cs;

                if ((cs += data[i++]) != 0x20) return;
                if ((cs += data[i++]) != 0x40) return;

                for(uint8_t k = 0; k < 28; ++k) {
                    cs += data[i++];
                }
                cs.lowByte(data[i++]);
                cs.highByte(data[i++]);
                if (cs) {
                    for(uint8_t k = 0; k < 18; ++k) {
                        decode(k);
                    }
                }
                else {
                    ++mErrorCount;
                }
            }
            static inline volatile bool mActive = false;
            static inline std::array<uint16_t, RC::Protokoll::IBus::V2::numberOfChannels> mChannels; // sbus
            static inline uint16_t mErrorCount = 0;
            static inline volatile etl::Event<Event> mEvent;
            static inline volatile State mState = State::Init;
            static inline External::Tick<systemTimer> mStateTick;
        };
    }
}
