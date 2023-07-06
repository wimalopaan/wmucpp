#pragma once

#pragma once

#include "mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu_traits.h"

#include <type_traits>
#include <concepts>

namespace Mcu::Stm {
    using namespace Units::literals;

    template<uint8_t N, typename Period, typename Prescaler, typename MCU = void> struct Timer;
    
    template<uint8_t N, uint16_t Per, uint16_t Pre, typename MCU>
    requires (N >=3) && (N <= 4)
    struct Timer<N, std::integral_constant<uint16_t, Per>, std::integral_constant<uint16_t, Pre>, MCU> {
        static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Timer<N, std::integral_constant<uint16_t, Per>, std::integral_constant<uint16_t, Pre>, MCU>>::value);
        static inline void init() {
            if constexpr(N == 3) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
            }
            else if constexpr(N == 4) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
            }
            else {
                static_assert(false);
            }
            mcuTimer->PSC = Pre;
            mcuTimer->ARR = Per;
            mcuTimer->CCR1 = Per / 4;
            mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC1M_Pos); // pwm1
            mcuTimer->CCMR1 |= TIM_CCMR1_OC1PE;
            mcuTimer->CCER |= TIM_CCER_CC1E;
            mcuTimer->EGR |= TIM_EGR_UG;
            mcuTimer->CR1 |= TIM_CR1_ARPE;
            mcuTimer->CR1 |= TIM_CR1_CEN;
        }        
        static inline void duty(const uint16_t d) {
            mcuTimer->CCR1 = d;            
        }
    };

    template<typename Per, typename Pre, G4xx MCU>
    struct Address<Timer<3, Per, Pre, MCU>> {
        static inline constexpr uintptr_t value = TIM3_BASE;        
    };
    template<typename Per, typename Pre, G4xx MCU>
    struct Address<Timer<4, Per, Pre, MCU>> {
        static inline constexpr uintptr_t value = TIM4_BASE;        
    };
}
