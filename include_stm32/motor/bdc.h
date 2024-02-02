#pragma once

#include "mcu/mcu.h"
#include "timer.h"
#include "units.h"
#include "concepts.h"
#include "mcu/mcu_traits.h"
#include "meta.h"
#include "mcu/alternate.h"
#include "etl/ranged.h"
#include "dsp.h"

#include <type_traits>
#include <concepts>
#include <algorithm>
#include <array>
#include <numeric>
#include <numbers>

namespace Mcu::Stm {
using namespace Units::literals;
namespace Motor {

template<uint8_t TimerNumber, typename Clock, typename MCU = DefaultMcu>
struct Bdc {
    static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Timer<TimerNumber, void, void, MCU>>::value);

    static inline constexpr uint16_t period = 1640;
    // static inline constexpr uint16_t period = 164;
    static inline constexpr uint16_t freq   = 20'000;

    static inline constexpr uint16_t prescaler = (Clock::config::frequency.value / (freq * period));

    // std::integral_constant<uint16_t, prescaler>::_;

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
        else {
            static_assert(false);
        }
        mcuTimer->PSC = prescaler;
        mcuTimer->ARR = period;
        mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC1M_Pos); // pwm1
        // mcuTimer->CCMR1 |= TIM_CCMR1_OC1PE;
        mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC2M_Pos); // pwm2
        // mcuTimer->CCMR1 |= TIM_CCMR1_OC2PE;
        mcuTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC3M_Pos); // pwm3
        // mcuTimer->CCMR2 |= TIM_CCMR2_OC3PE;
        mcuTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC4M_Pos); // pwm4
        // mcuTimer->CCMR2 |= TIM_CCMR2_OC4PE;
        mcuTimer->CCER |= TIM_CCER_CC1E;
        mcuTimer->CCER |= TIM_CCER_CC2E;

        mcuTimer->CCR1 = 0;
        mcuTimer->CCR2 = 0;
        mcuTimer->CCR3 = 0;
        mcuTimer->CR1 |= TIM_CR1_ARPE;

        mcuTimer->CR2 |= (0b110 << TIM_CR2_MMS_Pos); // ch3 as trigger-output

        mcuTimer->CR1 |= TIM_CR1_CEN;
    }

    static inline void duty(uint16_t v) {
        // v /= 10;
        mcuTimer->CCR1 = v;
        mcuTimer->CCR2 = v;
        mcuTimer->CCR3 = (9 * v) / 10; // trigger
    }

    constexpr static inline uint8_t trgo() {
        return 4; // tim3-trgo
    }
};

}
}
