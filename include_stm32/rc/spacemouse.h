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
#include "debug_pin.h"

namespace RC::Protokoll::SpaceMouse {
    using namespace std::literals::chrono_literals;
    template<uint8_t N, typename Config, typename MCU = DefaultMcu>
    struct Input {
        using clock = Config::clock;
        using systemTimer = Config::systemTimer;
        using debug = Config::debug;
        using pin = Config::pin;
        using tp = Config::tp;

        using axis_t = std::array<uint16_t, 6>;

        inline static constexpr uint16_t halfSpan = 360;
        inline static constexpr uint16_t midValue = 8192;
        inline static constexpr uint16_t maxValue = midValue + halfSpan;
        inline static constexpr uint16_t minValue = midValue - halfSpan;
        inline static constexpr uint8_t startSymbol = 0x96;
        inline static constexpr uint8_t endSymbol = 0x8D;

        struct UartConfig {
            using Clock = clock;
            using ValueType = uint8_t;
            static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::RxOnly;
            static inline constexpr uint32_t baudrate = 38'400;
            using DmaChComponent = Config::dmaChComponent;
            struct Rx {
                static inline constexpr bool enable = true;
                static inline constexpr size_t size = 16;
                static inline constexpr size_t idleMinSize = 4;
            };
            struct Isr {
                static inline constexpr bool idle = true;
            };
            using tp = Config::tp;
        };
        using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;
        static inline void init() {
            static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::RX>;
            IO::outl<debug>("# SpMouse init ", N);
            for(auto& v: mAxis) {
                v = midValue;
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
            IO::outl<debug>("# SpMouse reset ", N);
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
            return mAxis[ch];
        }
        static inline auto errorCount() {
            return mErrorCount;
        }
        private:
        static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t n) {
            if (data[0] != startSymbol) {
                return false;
            }
            if (data[15] != endSymbol) {
                return false;
            }
            if (n != 16) {
                return false;
            }
            return true;
        }
        static inline void readReply() {
            uart::readBuffer([](const auto& data){
                uint16_t cs = 0;
                for(uint8_t i = 0; i < (12 + 1); ++i) {
                    cs += data[i];
                }
                uint16_t csp = (data[12 + 1] << 7) + data[13 + 1];
                if (cs == csp) {
                    ++mPackagesCount;
                    decode(data);
                }
                else {
                    ++mErrorCount;
                }
            });
        }
        static inline void decode(const auto& data) {
            for(uint8_t axis = 0; auto& v: mAxis) {
                v = (data[2 * axis] << 7) + (data[2 * axis + 1]);
            }
        }
        static inline volatile bool mActive = false;
        static inline axis_t mAxis;
        static inline uint16_t mErrorCount = 0;
        static inline uint16_t mPackagesCount = 0;
        static inline volatile etl::Event<Event> mEvent;
        static inline volatile State mState = State::Init;
        static inline External::Tick<systemTimer> mStateTick;
    };
}
