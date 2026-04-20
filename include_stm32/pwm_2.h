/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <type_traits>
#include <concepts>
#include <array>
#include <cmath>

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/alternate.h"
#include "units.h"
#include "concepts.h"
#include "meta.h"

#include "timer.h"
#include "dma_2.h"
#include "output.h"

namespace Mcu::Stm {
    using namespace Units::literals;

#ifdef USE_MCU_STM_V2
    inline
#endif
    namespace V3 {
        namespace Pwm {
            namespace Reflection {
            }
                    
            template<uint8_t TimerNumber, typename Config, typename MCU = DefaultMcu>
            struct InOut {
                static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Timer<TimerNumber>>::value);
                using component_t = Mcu::Components::Timer<TimerNumber>;
                using clock = Config::clock;
                using debug = Config::debug;
                using dmaCh1Comp = Config::dmaChComponent1;

                struct Dma1Config {
                    using debug = Config::debug;
                    using controller = Mcu::Stm::Dma::Controller<dmaCh1Comp::controller::number_t::value>;
                    using value_t = uint16_t;
                    static inline constexpr bool memoryIncrement = false;
                    static inline constexpr bool circular = true;
                    struct Isr {
                        static inline constexpr bool txComplete = true;
                    };
                };
                using dmaCh1 = Mcu::Stm::Dma::V2::Channel<dmaCh1Comp::number_t::value, Dma1Config>;
                
                using pin_in_1 = Config::pin_in_1;
                static inline constexpr uint8_t pin_in_1_af = Mcu::Stm::AlternateFunctions::mapper_v<pin_in_1, InOut, Mcu::Stm::AlternateFunctions::CC<1>>;
                using pin_out_1 = Config::pin_out_1;
                static inline constexpr uint8_t pin_out_1_af = Mcu::Stm::AlternateFunctions::mapper_v<pin_out_1, InOut, Mcu::Stm::AlternateFunctions::CC<3>>;

                using pin_in_2 = Config::pin_in_2;
                using pin_out_2 = Config::pin_out_2;
                
                static inline constexpr uint16_t period = (1<<16) - 1;
                // static inline constexpr uint16_t period = 0;
                static inline constexpr uint32_t freq1  = 4'000'000;
                static inline constexpr uint16_t prescaler = (clock::config::frequency.value / freq1);
                // std::integral_constant<uint16_t, prescaler>::_;
                static inline constexpr uint32_t freq  = clock::config::frequency.value / prescaler;
                // std::integral_constant<uint32_t, freq>::_;

                static inline constexpr uint16_t onems = freq / 1000;
                static inline constexpr uint16_t mid   = onems + onems / 2;
                
                // std::integral_constant<uint16_t, onems>::_;
                
                static inline void reset() {
                    IO::outl<debug>("# PwmInOut reset");
                    const auto rcc = Mcu::Stm::Address<Mcu::Components::Rcc>::value;
                    if constexpr (TimerNumber == 3) {
                        rcc->APBRSTR1 = RCC_APBRSTR1_TIM3RST;
                        rcc->APBRSTR1 &= ~RCC_APBRSTR1_TIM3RST;
                    }
        #ifdef STM32G0B1xx
                    else if constexpr (TimerNumber == 4) {
                        rcc->APBRSTR1 = RCC_APBRSTR1_TIM4RST;
                        rcc->APBRSTR1 &= ~RCC_APBRSTR1_TIM4RST;
                    }
        #endif
                    else {
                        static_assert(false);
                    }
                    pin_in_1::analog();
                    pin_in_2::analog();
                    pin_out_1::analog();
                    pin_out_2::analog();
                }
                
// #ifdef STM32G0
                static inline void init() {
                    IO::outl<debug>("# PwmInOut init");
                    if constexpr (TimerNumber == 1) {
                        RCC->APBENR2 |= RCC_APBENR2_TIM1EN;
                    }
#ifdef RCC_APBENR1_TIM2EN
                    else if constexpr (TimerNumber == 2) {
                        RCC->APBENR1 |= RCC_APBENR1_TIM2EN;
                    }
#endif
                    else if constexpr (TimerNumber == 3) {
                        RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
                    }
                    else if constexpr (TimerNumber == 14) {
                        RCC->APBENR2 |= RCC_APBENR2_TIM14EN;
                    }
                    else if constexpr (TimerNumber == 17) {
                        RCC->APBENR2 |= RCC_APBENR2_TIM17EN;
                    }
                    else {
                        static_assert(false);
                    }
                    
                    mcuTimer->PSC = prescaler - 1;
                    mcuTimer->ARR = period;
                    // mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC1M_Pos); // pwm1
                    // mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC2M_Pos); // pwm2
                    mcuTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC3M_Pos); // pwm3
                    mcuTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC4M_Pos); // pwm4
                    // mcuTimer->CCER |= TIM_CCER_CC1E;
                    // mcuTimer->CCER |= TIM_CCER_CC2E;
                    mcuTimer->CCER |= TIM_CCER_CC3E;
                    mcuTimer->CCER |= TIM_CCER_CC4E;

                    if constexpr ((TimerNumber == 1) || (TimerNumber == 17)) {
                        mcuTimer->BDTR |= TIM_BDTR_MOE;
                        mcuTimer->CCER |= TIM_CCER_CC1E;
                    }
                    if constexpr ((TimerNumber == 17)) { // hack
                        mcuTimer->CCER |= TIM_CCER_CC1NE;
                    }
                    // mcuTimer->CCR1 = 0;
                    // mcuTimer->CCR2 = 0;
                    mcuTimer->CCR3 = mid;
                    mcuTimer->CCR4 = mid;
                    mcuTimer->CR1 |= TIM_CR1_ARPE;

#if 1
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
        
                    mcuTimer->DIER |= TIM_DIER_CC1DE; // dma enable
        
                    dmaCh1::init();
                    dmaCh1::startRead(1, (uint32_t)&mcuTimer->DMAR, &mData, Mcu::Stm::Timers::Properties<TimerNumber>::dmamux_src[0]);
                    
                    // MODIFY_REG(mcuTimer->DCR, TIM_DCR_DBL_Msk, 0x01 << TIM_DCR_DBL_Pos); // Burstlenth = 2
        
        //             static constexpr uint16_t ccr1Offset = offsetof(TIM_TypeDef, CCR1) / 4;
        //             static_assert(offsetof(TIM_TypeDef, CR1) == 0);
        //             static_assert(offsetof(TIM_TypeDef, CR2) == 4);
        
        //             MODIFY_REG(mcuTimer->DCR, TIM_DCR_DBA_Msk, ccr1Offset << TIM_DCR_DBA_Pos);
#endif
                    
                    mcuTimer->CR1 |= TIM_CR1_CEN;
        
                    pin_in_1::afunction(pin_in_1_af);
                    pin_in_1::template dir<Mcu::Input>();
                    
                    pin_out_1::afunction(pin_out_1_af);
                    pin_out_1::template dir<Mcu::Output>();
                    
                    pin_in_2::analog();
                    pin_out_2::analog();
                }
// #endif
                static inline void duty3(const uint16_t v) {
                    mcuTimer->CCR3 = v;
                }
                static inline void duty4(const uint16_t v) {
                    mcuTimer->CCR4 = v;
                }
                
            private:
                static inline bool mStartPositiv = true;
                static inline uint16_t mData = 0;
                
            };
        }
    }
}
