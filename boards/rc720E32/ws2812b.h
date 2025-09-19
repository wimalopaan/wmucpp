#pragma once

#include <cstdint>
#include <type_traits>
#include <concepts>
#include <array>

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/alternate.h"
#include "units.h"
#include "concepts.h"
#include "meta.h"

#include "timer.h"
#include "dma_2.h"

namespace External {
    template<uint8_t TimerNumber, typename Config, typename MCU>
    struct WS2812B {
        static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Timer<TimerNumber>>::value);
        using component_t = Mcu::Components::Timer<TimerNumber>;

        using dmaChWComp = Config::dmaChWComponent;
        using debug = Config::debug;
        using clock = Config::clock;
        using value_type = Mcu::Stm::Timers::Properties<TimerNumber>::value_type;

        static inline constexpr uint8_t channel = Config::channel;
        static inline constexpr uint8_t numLeds = Config::numLeds;
        static_assert(numLeds <= 64);

        static inline constexpr Units::hertz frequency{800'000};
        static inline constexpr uint16_t period = (clock::config::frequency.value / frequency.value); // 1250ns
        // using x = std::integral_constant<uint16_t, period>::_;

        static inline constexpr uint16_t t_low  = (period * 350) / 1250;
        static inline constexpr uint16_t t_high = period - t_low;
        // using x1 = std::integral_constant<uint16_t, t_low>::_;
        // using x2 = std::integral_constant<uint16_t, t_high>::_;

        struct ColorRGB {
            uint8_t red = 0;
            uint8_t green = 0;
            uint8_t blue = 0;
        };

        struct dmaWConfig {
            using controller = Mcu::Stm::Dma::Controller<dmaChWComp::controller::number_t::value>;
            using value_t = value_type;
            static inline constexpr bool memoryIncrement = true;
            struct Isr {
                static inline constexpr bool txComplete = true;
            };
        };
        using dmaChW = Mcu::Stm::Dma::V2::Channel<dmaChWComp::number_t::value, dmaWConfig>;

        static inline constexpr uint8_t number = dmaChW::number;

        enum class State : uint8_t {Idle, Transfer};
        enum class Event : uint8_t {None, StartTransfer, SendComplete};

        static inline void init() {
            using pin = Config::pin;
            static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, WS2812B<TimerNumber, Config, MCU>, Mcu::Stm::AlternateFunctions::CC<channel>>;
            IO::outl<debug>("# WS2812 init");
            Mcu::Arm::Atomic::access([]{
                mState = State::Idle;
                mActive = true;
            });
            Mcu::Stm::Timers::template powerUp<TimerNumber>();

            mcuTimer->PSC = 0;
            mcuTimer->ARR = period;

            dmaChW::init();

            if constexpr(channel == 1) {
                mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC1M_Pos); // pwm1
                mcuTimer->CCER |= TIM_CCER_CC1E;
            }
            else if constexpr(channel == 2) {
                mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC2M_Pos);
                mcuTimer->CCER |= TIM_CCER_CC2E;
            }
            else if constexpr(channel == 3) {
                mcuTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC3M_Pos);
                mcuTimer->CCER |= TIM_CCER_CC3E;
            }
            else if constexpr(channel == 4) {
                mcuTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC4M_Pos);
                mcuTimer->CCER |= TIM_CCER_CC4E;
            }
            else {
                static_assert(false);
            }
            mcuTimer->DIER |= TIM_DIER_UDE;
            mcuTimer->EGR |= TIM_EGR_UG;
            mcuTimer->CR1 |= TIM_CR1_ARPE;
            pin::afunction(af);
        }
        static inline void reset() {
            IO::outl<debug>("# WS2812 reset");
            Mcu::Arm::Atomic::access([]{
                Mcu::Stm::Timers::template reset<TimerNumber>();
                dmaChW::reset();
                mActive = false;
            });
            using pin = Config::pin;
            pin::analog();
        }
        static inline void sendColors() {
            IO::outl<debug>("# WS2812 sendColors");
            if (mActive) {
                mEvent = Event::StartTransfer;
            }
        }
        static inline void set(const uint8_t led, const ColorRGB color) {
            if (led < mColors.size()) {
                mColors[led] = color;
            }
        }
        static inline void on(const uint8_t led, const bool on) {
            if (led < mColors.size()) {
                const uint64_t mask = uint64_t{1} << led;
                if (on) {
                    mLedState |= mask;
                }
                else {
                    mLedState &= ~mask;
                }
                IO::outl<debug>("# WS2812 on: ", led, " ", (uint8_t)on, " ", mLedState, " ", (uint8_t)isOn(led));
            }
        }
        static inline bool isOn(const uint8_t led) {
            if (led < mColors.size()) {
                const uint64_t mask = uint64_t{1} << led;
                return mLedState & mask;
            }
            return false;
        }
        static inline void periodic() {
            switch(mState) {
                case State::Idle:
                mEvent.on(Event::StartTransfer, []{
                    fillData();
                    startDma();
                    startTimer();
                    mState = State::Transfer;
                    IO::outl<debug>("# WS2812 start transfer");
                });
                break;
            case State::Transfer:
                mEvent.on(Event::SendComplete, []{
                    mState = State::Idle;
                    IO::outl<debug>("# WS2812 idle");
                });
                break;
            }
        }
        static inline void ratePeriodic() {
        }
        struct Isr {
            static inline void onTransferComplete(const auto f) {
                if (mActive) {
                    stopTimer();
                    dmaChW::clearTransferCompleteIF();
                    f();
                    event(Event::SendComplete);
                }
            }
        };
        private:
        template<uint8_t Ch>
        static inline volatile uint32_t& ccrReg() {
            if constexpr(Ch == 1) {
                return mcuTimer->CCR1;
            }
            else if constexpr(Ch == 2) {
                return mcuTimer->CCR2;
            }
            else if constexpr(Ch == 3) {
                return mcuTimer->CCR3;
            }
            else if constexpr(Ch == 4) {
                return mcuTimer->CCR4;
            }
            else {
                static_assert(false);
            }
        }
        static inline void fillData() {
            uint16_t i = 0;
            for(uint16_t l = 0; l < mColors.size(); ++l) {
                if (isOn(l)) {
                    uint8_t mask = 1;
                    for(uint8_t b = 0; b < 8; ++b) {
                        if (mColors[l].red & mask) {
                            mData[i++] = t_high;
                        }
                        else {
                            mData[i++] = t_low;
                        }
                        mask <<= 1;
                    }
                    mask = 1;
                    for(uint8_t b = 0; b < 8; ++b) {
                        if (mColors[l].green & mask) {
                            mData[i++] = t_high;
                        }
                        else {
                            mData[i++] = t_low;
                        }
                        mask <<= 1;
                    }
                    mask = 1;
                    for(uint8_t b = 0; b < 8; ++b) {
                        if (mColors[l].blue & mask) {
                            mData[i++] = t_high;
                        }
                        else {
                            mData[i++] = t_low;
                        }
                        mask <<= 1;
                    }
                }
                else {
                    for(uint8_t b = 0; b < 8; ++b) {
                        mData[i++] = t_low;
                    }
                    for(uint8_t b = 0; b < 8; ++b) {
                        mData[i++] = t_low;
                    }
                    for(uint8_t b = 0; b < 8; ++b) {
                        mData[i++] = t_low;
                    }
                }
            }
        }
        static inline void startTimer() {
            ccrReg<TimerNumber>() = 0;
            mcuTimer->CR1 |= TIM_CR1_CEN;
        }
        static inline void stopTimer() {
            mcuTimer->CR1 &= ~TIM_CR1_CEN;
        }
        static inline void startDma() {
            dmaChW::startWrite(mData.size(), (uint32_t)& ccrReg<channel>(), &mData[0], Mcu::Stm::Timers::Properties<TimerNumber>::dmaUpdate_src);
        }
        static inline void event(const Event e) {
            mEvent = e;
        }
        static inline volatile etl::Event<Event> mEvent;
        static inline volatile State mState = State::Idle;
        static inline bool mActive = false;
        static inline std::array<ColorRGB, numLeds> mColors{};
        static inline std::array<volatile value_type, 3 * (numLeds + 1) * 8> mData{};
        static inline uint64_t mLedState{};
    };
}
