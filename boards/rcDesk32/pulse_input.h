/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include <chrono>

#include "mcu/alternate.h"
#include "output.h"
#include "etl/util.h"
#include "etl/event.h"
#include "etl/fixedvector.h"
#include "timer.h"
#include "rc/rc_2.h"

using namespace std::literals::chrono_literals;

namespace Pulse {

    template<typename T = int32_t>
    struct ExpMean {
        ExpMean() = default;
        explicit ExpMean(const T n, const T d) : mN{n}, mD{d} {}
        inline T process(const T v) {
            mValue = (mValue * mN) / mD + (v * (mD - mN)) / mD;
            return mValue;
        }
        inline T value() const {
            return mValue;
        }
        inline void setN(const T n) {
            mN = n;
        }
        inline void set(const T v) {
            mValue = v;
        }
        private:
        T mN = 90;
        T mD = 100;
        T mValue = 0;
    };


    template<auto TimerNumber, typename Config, typename MCU = DefaultMcu>
    struct CppmIn {
        using pin = Config::pin;
        using clock = Config::clock;
        using systemTimer = Config::timer;
        using tp = Config::tp;
        using debug = Config::debug;

        using switchCallback = Config::switchCallback;

        static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Timer<TimerNumber>>::value);
        static inline constexpr uint8_t timerNumber = TimerNumber;
        using component_t = Mcu::Components::Timer<TimerNumber>;

        static inline constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, CppmIn<TimerNumber, Config, MCU>, Mcu::Stm::AlternateFunctions::CC<1>>;

        static inline constexpr uint16_t onems = RC::Protokoll::SBus::V2::amp;
        static inline constexpr uint16_t mid = onems + onems / 2;
        static inline constexpr uint16_t period = onems * 20;
        static inline constexpr uint16_t prescaler = (clock::config::frequency.value / period) / 50;

        static inline constexpr uint16_t sd = onems / 3;
        static inline constexpr uint16_t t1 = RC::Protokoll::SBus::V2::mid - sd / 2;
        static inline constexpr uint16_t t2 = RC::Protokoll::SBus::V2::mid + sd / 2;

        using timer_value_t = std::conditional_t<(timerNumber == 2), uint32_t, uint16_t>;

        using dmaChComponent = Config::dmaCh;
        struct dmaChConfig {
            using controller = Mcu::Stm::Dma::Controller<dmaChComponent::controller::number_t::value>;
            using value_t = timer_value_t;
            static inline constexpr bool memoryIncrement = true;
        };
        using dmaCh = Mcu::Stm::Dma::V2::Channel<dmaChComponent::number_t::value, dmaChConfig>;

        static inline void reset() {
            IO::outl<debug>("# CppmIn reset");
            Mcu::Stm::Timers::reset<2>();
            pin::analog();
            mActive = false;
        }
        static inline void init() {
            IO::outl<debug>("# CppmIn init");
            Mcu::Stm::Timers::powerUp<2>();
            mActive = true;

            setExpN(90);

            mcuTimer->PSC = prescaler;
            mcuTimer->ARR = period;

            MODIFY_REG(mcuTimer->TISEL, TIM_TISEL_TI1SEL_Msk, 0); // input CH1
            MODIFY_REG(mcuTimer->CCMR1, TIM_CCMR1_CC1S_Msk, 0b01 << TIM_CCMR1_CC1S_Pos); // TI1 -> IC1
            MODIFY_REG(mcuTimer->CCMR1, TIM_CCMR1_CC2S_Msk, 0b10 << TIM_CCMR1_CC2S_Pos); // TI1 -> IC2
            MODIFY_REG(mcuTimer->CCMR1, TIM_CCMR1_IC1PSC_Msk, 0b00 << TIM_CCMR1_IC1PSC_Pos); // no prescaler

            if constexpr(Config::startPositive) {
                mcuTimer->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); // IC1: positive
                mcuTimer->CCER &= ~TIM_CCER_CC2NP; // IC2: negative
                mcuTimer->CCER |= TIM_CCER_CC2P;
            }
            else {
                mcuTimer->CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP); // IC2: positive
                mcuTimer->CCER &= ~TIM_CCER_CC1NP; // IC1: negative
                mcuTimer->CCER |= TIM_CCER_CC1P;
            }
            MODIFY_REG(mcuTimer->SMCR, TIM_SMCR_TS_Msk, TIM_SMCR_TS_0 | TIM_SMCR_TS_2); // TI1FP1 as Trigger
            MODIFY_REG(mcuTimer->SMCR, TIM_SMCR_SMS_Msk, 0b0100); // slave mode: reset

            mcuTimer->CCER |= TIM_CCER_CC1E;
            mcuTimer->CCER |= TIM_CCER_CC2E;

            mcuTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC3M_Pos); // pwm3
            mcuTimer->CCMR2 |= TIM_CCMR2_OC3PE;
            mcuTimer->CCER |= TIM_CCER_CC3E;
            mcuTimer->CCR3 = onems * 4; // pause between pulse trains

            // mcuTimer->DIER |= TIM_DIER_CC1IE; // Test
            // mcuTimer->DIER |= TIM_DIER_CC2IE; // Test
            mcuTimer->DIER |= TIM_DIER_CC3IE;

            mcuTimer->DIER |= TIM_DIER_CC1DE; // dma enable

            dmaCh::init();
            dmaCh::startRead(2 * mData.size(), (uint32_t)&mcuTimer->DMAR, &mData[0].first, Mcu::Stm::Timers::Properties<TimerNumber>::dmamux_src[0]);

            MODIFY_REG(mcuTimer->DCR, TIM_DCR_DBL_Msk, 0x01 << TIM_DCR_DBL_Pos); // Burstlenth = 2

            static constexpr uint16_t ccr1Offset = offsetof(TIM_TypeDef, CCR1) / 4;
            static_assert(offsetof(TIM_TypeDef, CR1) == 0);
            static_assert(offsetof(TIM_TypeDef, CR2) == 4);

            MODIFY_REG(mcuTimer->DCR, TIM_DCR_DBA_Msk, ccr1Offset << TIM_DCR_DBA_Pos);

            mcuTimer->CR1 |= TIM_CR1_CEN;

            pin::afunction(af);
            pin::template pullup<true>();
        }
        enum class Event : uint8_t {None, EndOfFrame};
        struct Isr {
            static inline void onCapture(const auto f) {
                if (mActive) {
                    if (mcuTimer->SR & TIM_SR_CC1IF) {
                        f();
                    }
                    if (mcuTimer->SR & TIM_SR_CC2IF) {
                        f();
                    }
                    if (mcuTimer->SR & TIM_SR_CC3IF) {
                        f();
                        mEvent = Event::EndOfFrame;
                    }
                    mcuTimer->SR = 0;
                }
            }
        };
        static inline bool mIsCalibrating = false;
        static inline void startCalibrate() {
            resetCalibrationData();
            mIsCalibrating = true;
        }
        static inline void stopCalibrate() {
            mIsCalibrating = false;
        }
        static inline bool isCalibrating() {
            return mIsCalibrating;
        }
        static inline void setExpN(const uint8_t n) {
            for(auto& exp: mValues) {
                exp.setN(n);
            }
        }
        template<bool Positive = true, bool enable = true>
        static inline void startPositive() {
            if constexpr(enable) {
                mcuTimer->CR1 &= ~TIM_CR1_CEN;
            }
            if constexpr(Positive) {
                mcuTimer->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); // IC1: positive
                mcuTimer->CCER &= ~TIM_CCER_CC2NP; // IC2: negative
                mcuTimer->CCER |= TIM_CCER_CC2P;
            }
            else {
                mcuTimer->CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP); // IC2: positive
                mcuTimer->CCER &= ~TIM_CCER_CC1NP; // IC1: negative
                mcuTimer->CCER |= TIM_CCER_CC1P;
            }
            if constexpr(enable) {
                mcuTimer->CR1 |= TIM_CR1_CEN;
            }
        }
        static inline void periodic() {
            mEvent.on(Event::EndOfFrame, []{
                decode();
                startDmaSequence();
            });
        }
        static inline const auto& switches() {
            return mSwChannels;
        }
        static inline const auto& values() {
            return mValues;
        }
        static inline uint16_t value(const uint8_t ch) {
            if (ch < 8) {
                return mValues[ch].value();
            }
            else {
                return RC::Protokoll::SBus::V2::mid;
            }
        }
        private:
        static inline void resetCalibrationData() {
            for(MinMax& mm: mMinMax) {
                mm = MinMax{};
            }
        }
        struct SwitchData {
            uint8_t channel = 0;
            uint8_t index = 0;
            std::array<uint8_t, 8> values{};
        };
        static inline bool isSwitchChannel(const uint8_t ch) {
            for(const SwitchData& swd: mSwChannels) {
                if (ch == swd.channel) {
                    return true;
                }
            }
            return false;
        }
        static inline void decodeChannels() {
            for(uint8_t i = 0; i < 8; ++i) {
                const uint8_t index = i + 1;
                if (mPeriod) {
                    const uint16_t s = mData[index].first - onems + RC::Protokoll::SBus::V2::min;
                    if (isSwitchChannel(i)) {
                        mValues[i].set(s);
                    }
                    else {
                        mValues[i].process(std::clamp(s, RC::Protokoll::SBus::V2::min, RC::Protokoll::SBus::V2::max));
                    }
                }
                else {
                    const uint16_t s = mData[index].second - onems + RC::Protokoll::SBus::V2::min;
                    mValues[i].process(std::clamp(s, RC::Protokoll::SBus::V2::min, RC::Protokoll::SBus::V2::max));
                }
            }
        }
        static inline void decodeSwitches() {
            bool changed = false;
            for(auto& sd : mSwChannels) {
                if (value(sd.channel) < RC::Protokoll::SBus::V2::min) {
                    sd.index = 0;
                }
                else {
                    if (sd.index < sd.values.size()) {
                        if (value(sd.channel) < t1) {
                            if (sd.values[sd.index] != 1) {
                                changed = true;
                                sd.values[sd.index] = 1;
                            }
                        }
                        else if (value(sd.channel) > t2) {
                            if (sd.values[sd.index] != 2) {
                                changed = true;
                                sd.values[sd.index] = 2;
                            }
                        }
                        else {
                            if (sd.values[sd.index] != 0) {
                                changed = true;
                                sd.values[sd.index] = 0;
                            }
                        }
                    }
                    ++sd.index;
                }
            }
            if (changed) {
                switchCallback::update();
            }
        }
        static inline void decode() {
            decodeChannels();
            decodeSwitches();
        }
        static inline void startDmaSequence() {
            dmaCh::startRead(2 * mData.size(), (uint32_t)&mcuTimer->DMAR, &mData[0].first, Mcu::Stm::Timers::Properties<TimerNumber>::dmamux_src[0]);
        }
        static inline std::array<std::pair<volatile timer_value_t, volatile timer_value_t>, 16> mData; // first pair is invalid (old counter values)
        static inline bool mPeriod = true;
        static inline volatile bool mActive = false;
        static inline std::array<ExpMean<uint16_t>, 16> mValues;
        static inline etl::FixedVector<SwitchData, 4> mSwChannels{SwitchData{6, 0}, SwitchData{7, 0}};
        static inline etl::Event<Event> mEvent;
        static inline std::array<MinMax, 16> mMinMax{};
    };
}
