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
#include "timer.h"

using namespace std::literals::chrono_literals;

namespace Pulse {
    template<auto TimerNumber, typename Config, typename MCU = DefaultMcu>
    struct CppmIn {
        using pin = Config::pin;
        using clock = Config::clock;
        using systemTimer = Config::timer;
        using dmaCh = Config::dmaCh;
        using tp = Config::tp;
        using debug = Config::debug;

        static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Timer<TimerNumber>>::value);

        static inline constexpr uint8_t timerNumber = TimerNumber;

        using component_t = Mcu::Components::Timer<TimerNumber>;

        static inline constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, CppmIn<TimerNumber, Config, MCU>, Mcu::Stm::AlternateFunctions::CC<1>>;

        static inline constexpr uint16_t onems = 1640;
        static inline constexpr uint16_t mid = onems + onems / 2;
        static inline constexpr uint16_t period = onems * 20;
        static inline constexpr uint16_t prescaler = (clock::config::frequency.value / period) / 50;

        static inline void reset() {
            IO::outl<debug>("# CppmIn reset");
            const auto rcc = Mcu::Stm::Address<Mcu::Components::Rcc>::value;
#ifdef STM32G0B1xx
            if constexpr (TimerNumber == 4) {
                rcc->APBRSTR1 = RCC_APBRSTR1_TIM4RST;
                rcc->APBRSTR1 &= ~RCC_APBRSTR1_TIM4RST;
            }
#endif
            else {
                static_assert(false);
            }
            pin::analog();
        }
        static inline void init() {
            IO::outl<debug>("# CppmIn init");
#ifdef STM32G0B1xx
            if constexpr (TimerNumber == 4) {
                RCC->APBENR1 |= RCC_APBENR1_TIM4EN;
            }
#endif
            else {
                static_assert(false);
            }
            mcuTimer->PSC = prescaler;
            mcuTimer->ARR = period;

            MODIFY_REG(mcuTimer->TISEL, TIM_TISEL_TI1SEL_Msk, 0); // input CH1
            MODIFY_REG(mcuTimer->CCMR1, TIM_CCMR1_CC1S_Msk, 0b01 << TIM_CCMR1_CC1S_Pos); // TI1 -> IC1
            MODIFY_REG(mcuTimer->CCMR1, TIM_CCMR1_CC2S_Msk, 0b10 << TIM_CCMR1_CC2S_Pos); // TI1 -> IC2
            if (mStartPositiv) {
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
            mcuTimer->CCR3 = onems * 4;

            // mcuTimer->DIER |= TIM_DIER_CC1IE; // Test
            // mcuTimer->DIER |= TIM_DIER_CC2IE; // Test
            mcuTimer->DIER |= TIM_DIER_CC3IE;

            mcuTimer->DIER |= TIM_DIER_CC1DE; // dma enable

            dmaCh::init();
            dmaCh::template msize<uint16_t>();
            dmaCh::template psize<uint16_t>();
            dmaCh::mcuDmaChannel->CNDTR = 2 * mData.size();
            dmaCh::mcuDmaChannel->CCR |= DMA_CCR_MINC;
            dmaCh::mcuDmaChannel->CPAR = (uint32_t)&mcuTimer->DMAR;
            dmaCh::mcuDmaChannel->CMAR = (uint32_t)&mData[0];
            MODIFY_REG(dmaCh::mcuDmaMux->CCR, DMAMUX_CxCR_DMAREQ_ID_Msk,
                       Mcu::Stm::Timers::Properties<TimerNumber>::dmamux_src[0] << DMAMUX_CxCR_DMAREQ_ID_Pos);

            MODIFY_REG(mcuTimer->DCR, TIM_DCR_DBL_Msk, 0x01 << TIM_DCR_DBL_Pos); // Burstlenth = 2

            static constexpr uint16_t ccr1Offset = offsetof(TIM_TypeDef, CCR1) / 4;
            static_assert(offsetof(TIM_TypeDef, CR1) == 0);
            static_assert(offsetof(TIM_TypeDef, CR2) == 4);

            MODIFY_REG(mcuTimer->DCR, TIM_DCR_DBA_Msk, ccr1Offset << TIM_DCR_DBA_Pos);

            mcuTimer->CR1 |= TIM_CR1_CEN;

            pin::template dir<Mcu::Input>();
            pin::afunction(af);
        }
        static inline void onCapture(const auto f) {
            if (mcuTimer->SR & TIM_SR_CC1IF) {
                f();
            }
            if (mcuTimer->SR & TIM_SR_CC2IF) {
                f();
            }
            if (mcuTimer->SR & TIM_SR_CC3IF) {
                f();
                startDmaSequence();
            }
            mcuTimer->SR = 0;
        }
        static inline void positive(const bool positiv, const bool period = true) {
            mcuTimer->CR1 &= ~TIM_CR1_CEN;
            mStartPositiv = positiv;
            mPeriod = period;
            if (mStartPositiv) {
                mcuTimer->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); // IC1: positive
                mcuTimer->CCER &= ~TIM_CCER_CC2NP; // IC2: negative
                mcuTimer->CCER |= TIM_CCER_CC2P;
            }
            else {
                mcuTimer->CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP); // IC2: positive
                mcuTimer->CCER &= ~TIM_CCER_CC1NP; // IC1: negative
                mcuTimer->CCER |= TIM_CCER_CC1P;
            }
            mcuTimer->CR1 |= TIM_CR1_CEN;
        }
        static inline void update() {
        }
        static inline void periodic() {
        }
        static inline void ratePeriodic() {
        }
        static inline uint16_t value(const uint8_t ch) {
            if (ch < 8) {
                const uint8_t index = ch + 1;
                if (mPeriod) {
                    const int s = mData[index].first - onems + 172;
                    return std::clamp(s, 172, 1812);
                }
                else {
                    const int s = mData[index].second - onems + 172;
                    return std::clamp(s, 172, 1812);
                }
            }
            else {
                return 992;
            }
        }
        private:
        static inline void startDmaSequence() {
            if (!mPeriod) {
                const uint16_t c = 2 * mData.size() - dmaCh::mcuDmaChannel->CNDTR;
                mData[c/2].second = mData[0].second;
            }
            dmaCh::count(2 * mData.size());
        }
        static inline std::array<std::pair<volatile uint16_t, volatile uint16_t>, 16> mData; // first pair is invalid (old counter values)
        static inline bool mStartPositiv = false;
        static inline bool mPeriod = true;
    };
}
