#pragma once

#include <cstring>

#include "meta.h"

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "clock.h"
#include "pwm.h"
#include "timer.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "gpio.h"
#include "tick.h"

namespace Mcu::Stm {
    using namespace Units::literals;

    namespace Cppm {

        template<uint8_t TimerNumber, typename DmaChannel, typename Clock, typename MCU = DefaultMcu>
        struct Generator {
            static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Timer<TimerNumber>>::value);

            using dmaChannel = DmaChannel;

            using value_type = uint16_t;
            using timer_value_type = Mcu::Stm::Timers::Properties<TimerNumber>::value_type;

            static inline constexpr uint16_t onems = 1000;
            static inline constexpr uint16_t halfms = onems / 2;
            static inline constexpr uint16_t frame = 22.0f * onems;
            static inline constexpr uint8_t  channels = 8;
            static inline constexpr uint8_t  pulses = channels + 1;

            static inline constexpr uint16_t prescaler = (Clock::config::frequency.value / 1'000'000) - 1;

            static inline void init() {
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
                mcuTimer->PSC = prescaler;
                mcuTimer->ARR = onems + halfms - 1;
                mcuTimer->CCR1 = halfms - 1;
                mcuTimer->CCR2 = 3 * onems + halfms;

                for(uint8_t i {0}; i < channels; ++i) {
                    setNormalized(i, 500);
                }

                dmaChannel::init();
                dmaChannel::template msize<value_type>();
                dmaChannel::template psize<timer_value_type>();

                dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_MINC;
                dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_CIRC;
                dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_DIR;
                dmaChannel::mcuDmaChannel->CNDTR = pulses;
                dmaChannel::mcuDmaChannel->CPAR = (uint32_t)&mcuTimer->ARR;
                dmaChannel::mcuDmaChannel->CMAR = (uint32_t)&arr[0];
                // dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_TCIE; // for test
                dmaChannel::enable();

                dmaChannel::mcuDmaMux->CCR |= Mcu::Stm::Timers::Properties<TimerNumber>::dmaUpdate_src & DMAMUX_CxCR_DMAREQ_ID_Msk;
                dmaChannel::mcuDmaMux->CCR |= DMAMUX_CxCR_EGE;

                // Enable Timer, DMA

                mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC1M_Pos); // pwm1
                mcuTimer->CCER |= TIM_CCER_CC1E;
                mcuTimer->CCER |= TIM_CCER_CC1P;
                mcuTimer->DIER |= TIM_DIER_UDE;

                mcuTimer->DIER |= TIM_DIER_UIE; // start of pulse
                mcuTimer->DIER |= TIM_DIER_CC1IE; // end of pulse
                mcuTimer->DIER |= TIM_DIER_CC2IE; // sync pulse

                mcuTimer->EGR |= TIM_EGR_UG;
                // mcuTimer->CR1 |= TIM_CR1_URS;
                mcuTimer->CR1 |= TIM_CR1_ARPE;
                mcuTimer->CR1 |= TIM_CR1_CEN;
            }
            // value [0,1000]
            static inline void setNormalized(uint8_t channel, uint16_t value) {
                if (channel >= channels) {
                    return;
                }
                arr[channel] = value + onems - 1;
                uint32_t sum = arr[0];
                for(uint8_t i = 1; i < channels; ++i) {
                    sum += arr[i];
                }
                arr[pulses - 1] = frame - sum - 1;
            }
            // static inline void set(uint8_t channel, uint16_t value) {

            // }

            // private:
            static inline std::array<volatile value_type, pulses> arr{};
        };
    }
}
