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
#include "usart_2.h"

namespace RC::Protokoll::SBus {
    namespace V2 {
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
