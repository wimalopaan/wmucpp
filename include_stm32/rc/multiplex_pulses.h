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

#include <cstring>

#include "meta.h"

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "clock.h"
#include "dma_2.h"
#include "timer.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "gpio.h"
#include "tick.h"

namespace Mcu::Stm {
    using namespace Units::literals;
    namespace Cppm {
        template<uint8_t TimerNumber, typename Config, typename MCU = DefaultMcu>
        struct MultiplexGenerator {
            using component_t = Mcu::Components::Timer<TimerNumber>;

            static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Timer<TimerNumber>>::value);
            using clock = Config::clock;
            using value_type = Mcu::Stm::Timers::Properties<TimerNumber>::value_type;

            using dmaChComponent = Config::dmaCh;
            struct dmaChConfig;
            using dmaCh = Mcu::Stm::Dma::V2::Channel<dmaChComponent::number_t::value, dmaChConfig>;
            struct dmaChConfig {
                using debug = Config::debug;
                using controller = Mcu::Stm::Dma::Controller<dmaChComponent::controller::number_t::value>;
                using value_t = value_type;
                static inline constexpr bool memoryIncrement = true;
                static inline constexpr bool circular = true;
            };

            static inline constexpr uint8_t  channel = Config::channel;
            static inline constexpr uint16_t onems = 1000;
            static inline constexpr uint16_t halfms = onems / 2;
            static inline constexpr uint16_t off = onems + halfms;
            static inline constexpr uint16_t amp = 0.8f * halfms;
            static inline constexpr uint16_t sync = 2.1f * onems;
            static inline constexpr uint16_t frame = 22.0f * onems;
            static inline constexpr uint8_t  channels = 8;
            static inline constexpr uint8_t  pulses = channels + 1;

            static inline constexpr uint16_t prescaler = (clock::config::frequency.value / 1'000'000) - 1;

            static inline void init() {
#ifdef STM32G4
                if constexpr (TimerNumber == 1) {
                    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
                }
                else if constexpr (TimerNumber == 2) {
                    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
                }
                else if constexpr (TimerNumber == 3) {
                    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
                }
                else if constexpr (TimerNumber == 4) {
                    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
                }
                else if constexpr (TimerNumber == 16) {
                    RCC->APB1ENR1 |= RCC_APB2ENR_TIM16EN;
                }
                else {
                    static_assert(false);
                }
#endif
#ifdef STM32G0
                if constexpr (TimerNumber == 1) {
                    RCC->APBENR2 |= RCC_APBENR2_TIM1EN;
                }
                else if constexpr (TimerNumber == 3) {
                    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
                }
                // no DMA
                // else if constexpr (TimerNumber == 14) {
                //     RCC->APBENR2 |= RCC_APBENR2_TIM14EN;
                // }
                else if constexpr (TimerNumber == 17) {
                    RCC->APBENR2 |= RCC_APBENR2_TIM17EN;
                }
                else {
                    static_assert(false);
                }
#endif
                mcuTimer->PSC = prescaler;
                mcuTimer->ARR = frame;
                ccrReg<channel>() = onems + halfms;

                for(uint8_t i = 0; i < channels; ++i) {
                    arr[i] = off;
                }
                for(uint8_t i = channels; i < pulses; ++i) {
                    arr[i] = sync;
                }
                dmaCh::init();
                dmaCh::startWrite(pulses, (uint32_t)& ccrReg<channel>(), &arr[0], Mcu::Stm::Timers::Properties<TimerNumber>::dmaUpdate_src);

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
                mcuTimer->CR1 |= TIM_CR1_CEN;
            }
            static inline void set(const uint8_t channel, const uint8_t state) {
                if (channel >= 8) return;
                uint16_t v = off;
                if (state == 1) {
                    v += amp;
                }
                else if (state == 2) {
                    v -= amp;
                }
                arr[channel] = v;
            }
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
            static inline std::array<volatile value_type, pulses> arr{};
        };
    }
}
