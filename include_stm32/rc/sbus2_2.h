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

#include "byte.h"
#include "etl/algorithm.h"
#include "etl/ranged.h"
#include "units.h"
#include "tick.h"

#include "sbus2_types.h"

// Attention: some slots have swapped bytes

// https://github.com/neoxic/ESCape32/blob/master/src/io.c#L662-L665
// https://github.com/BrushlessPower/SBUS2-Telemetry/blob/master/src/SBUS2.cpp#L69

namespace RC::Protokoll::SBus2 {
    using namespace etl::literals;
    using namespace std::literals::chrono_literals;

    namespace V4 {
        template<auto N, typename Config, typename MCU>
        struct Master {
            using clock = Config::clock;
            using debug = Config::debug;
            using dmaChRW = Config::dmaChRW;
            using systemTimer = Config::systemTimer;
            using adapter = Config::adapter;
            using pin = Config::pin;

            struct UartConfig {
                using Clock = clock;
                using ValueType = uint8_t;
                static inline constexpr size_t size = 25;
                static inline constexpr size_t minSize = 4;
                using DmaChannelWrite = dmaChRW;
                using DmaChannelRead = dmaChRW;
                static inline constexpr bool useDmaTCIsr = false;
                static inline constexpr bool useIdleIsr = false;
                static inline constexpr bool useRxToIsr = false;
                static inline constexpr uint16_t rxToCount = 0;
                using Adapter = void;
                using Debug = struct {
                using tp = Config::tp;
            };
        };

        // private:
        using uart = Mcu::Stm::V2::Uart<N, UartConfig, MCU>;
        // public:
        static inline constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;

        static inline void update() {
            std::copy(std::begin(adapter::values()), std::end(adapter::values()), std::begin(output));
        }
        static inline void set(const uint8_t channel, const uint16_t value) {
            output[channel] = value;
        }
        static inline void init() {
            IO::outl<debug>("# Sbus2 ", N, " init");
            for(auto& v : output) {
                v = sbus_mid;
            }
            __disable_irq();
            uart::init();
            uart::template rxEnable<false>();
            uart::baud(100'000);
            uart::template halfDuplex<true>();
            uart::parity(true);
            uart::invert(true);
            uart::template enableTCIsr<true>();
            mActive = true;
            mState = State::SendFrame;
            mEvent = Event::None;
            __enable_irq();

            pin::afunction(af);
            pin::template pulldown<true>();
        }
        static inline void reset() {
            IO::outl<debug>("# Sbus2 ", N, " reset");
            __disable_irq();
            mActive = false;
            uart::reset();
            __enable_irq();
            pin::analog();
        }

        static constexpr External::Tick<systemTimer> timeoutTicks{ 14ms };

        enum class State : uint8_t { SendFrame, WaitForSlot, ReceiveSlots };
        enum class Event : uint8_t { None, SendComplete };

        static inline void event(const Event e) {
            mEvent = e;
        }
        static inline void activateSBus2(const bool a) {
            mUseSbus2 = a;
        }
        static inline void onTransferComplete(const auto f) {
            if (mActive) {
                uart::onTransferComplete(f);
            }
        }

        // https://github.com/neoxic/ESCape32/blob/master/src/io.c#L662-L665
        // https://github.com/BrushlessPower/SBUS2-Telemetry/blob/master/src/SBUS2.cpp#L69
        static inline void slotReceived() { // isr
            static constexpr std::array<uint8_t, 8> subNumbers{0, 4, 2, 6, 1, 5, 3, 7};
            const uint8_t slotStartByte = uart::readBuffer()[0];
            if ((slotStartByte & 0x1f) == response[mRequestIndex]) {
                const uint8_t subIndex = (slotStartByte >> 5) & 0x07;
                const uint8_t subNumber = subNumbers[subIndex];
                const uint8_t slotNumber = ((mRequestIndex * 8) + subNumber);
                mSlots[slotNumber] = (uart::readBuffer()[1] << 8) + uart::readBuffer()[2];
                if (subNumber < 7) {
                    dmaChRW::count(3); // with enable
                }
            }
            else {
                mSlotErrors = mSlotErrors + 1;
            }
        }
        inline static void periodic() {
        }
        inline static void ratePeriodic() {
            const auto oldState = mState;
            ++mStateTicks;
            const Event e = std::exchange(mEvent, Event::None);
            switch (mState) {
            case State::SendFrame:
                mRequestIndex = mRequestIndex + 1;
                if (mRequestIndex >= request.size()) {
                    mRequestIndex = 0;
                }
                fillFrame();
                mState = State::WaitForSlot;
                break;
            case State::WaitForSlot:
                if (e == Event::SendComplete) {
                    mState = State::ReceiveSlots;
                }
                break;
            case State::ReceiveSlots:
                mStateTicks.on(timeoutTicks, [] {
                    mState = State::SendFrame;
                });
                break;
            }
            if (mState != oldState) {
                mStateTicks.reset();
                switch(mState) {
                case State::SendFrame:
                    break;
                case State::WaitForSlot:
                    uart::template rxEnable<false>();
                    uart::template txEnable<true>();
                    uart::startSend(25);
                    break;
                case State::ReceiveSlots:
                    // debug::tp::set();
                    if (mUseSbus2) {
                        uart::template rxEnable<true>();
                        uart::template txEnable<false>();
                        dmaChRW::count(3); // with enable
                    }
                    // debug::tp::reset();
                    break;
                }
            }
        }
        static inline uint16_t errors() {
            return mSlotErrors;
        }
        static inline const auto& slots() {
            return mSlots;
        }
        private:
        static inline void fillFrame() {
            auto outFrame = uart::outputBuffer();
            outFrame[0] = start_byte;
            outFrame[1] = (output[0] & 0x07FF);
            outFrame[2] = ((output[0] & 0x07FF) >> 8 | (output[1] & 0x07FF) << 3);
            outFrame[3] = ((output[1] & 0x07FF) >> 5 | (output[2] & 0x07FF) << 6);
            outFrame[4] = ((output[2] & 0x07FF) >> 2);
            outFrame[5] = ((output[2] & 0x07FF) >> 10 | (output[3] & 0x07FF) << 1);
            outFrame[6] = ((output[3] & 0x07FF) >> 7 | (output[4] & 0x07FF) << 4);
            outFrame[7] = ((output[4] & 0x07FF) >> 4 | (output[5] & 0x07FF) << 7);
            outFrame[8] = ((output[5] & 0x07FF) >> 1);
            outFrame[9] = ((output[5] & 0x07FF) >> 9 | (output[6] & 0x07FF) << 2);
            outFrame[10] = ((output[6] & 0x07FF) >> 6 | (output[7] & 0x07FF) << 5);
            outFrame[11] = ((output[7] & 0x07FF) >> 3);
            outFrame[12] = ((output[8] & 0x07FF));
            outFrame[13] = ((output[8] & 0x07FF) >> 8 | (output[9] & 0x07FF) << 3);
            outFrame[14] = ((output[9] & 0x07FF) >> 5 | (output[10] & 0x07FF) << 6);
            outFrame[15] = ((output[10] & 0x07FF) >> 2);
            outFrame[16] = ((output[10] & 0x07FF) >> 10 | (output[11] & 0x07FF) << 1);
            outFrame[17] = ((output[11] & 0x07FF) >> 7 | (output[12] & 0x07FF) << 4);
            outFrame[18] = ((output[12] & 0x07FF) >> 4 | (output[13] & 0x07FF) << 7);
            outFrame[19] = ((output[13] & 0x07FF) >> 1);
            outFrame[20] = ((output[13] & 0x07FF) >> 9 | (output[14] & 0x07FF) << 2);
            outFrame[21] = ((output[14] & 0x07FF) >> 6 | (output[15] & 0x07FF) << 5);
            outFrame[22] = ((output[15] & 0x07FF) >> 3);
            outFrame[23] = (mFlagsAndSwitches); // Flags byte
            if (mUseSbus2) {
                outFrame[24] = (request[mRequestIndex]); // Telem-Request
            }
            else {
                outFrame[24] = 0x04;
            }
        }
        static inline volatile bool mActive = false;
        static inline volatile bool mUseSbus2 = false;
        static inline volatile uint16_t mSlotErrors = 0;
        static inline std::array<volatile uint16_t, 32> mSlots{};
        static inline constexpr std::array<uint8_t, 4> response{ 0x03, 0x13, 0x0b, 0x1b };
        static inline constexpr std::array<uint8_t, 4> request{ 0x04, 0x14, 0x24, 0x34 };
        static inline volatile uint8_t mRequestIndex{ 0 };
        static inline State mState{ State::SendFrame };
        static inline uint8_t mFlagsAndSwitches{};
        static inline std::array<uint16_t, 16> output;
        static inline External::Tick<systemTimer> mStateTicks{};
        static inline volatile Event mEvent = Event::None;
    };
}
}
