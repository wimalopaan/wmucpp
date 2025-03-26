#pragma once

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

namespace External {
    using namespace Units::literals;

    template <uint8_t N, typename Config, typename MCU = DefaultMcu>
    struct Encoder {
        static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Timer<N>>::value);
        using component_t = Mcu::Components::Timer<N>;

        using pin_a = Config::pin_a;
        using pin_b = Config::pin_b;
        using pin_t = Config::pin_t;

#ifdef STM32G0
        static inline void init() {
            if constexpr(N == 1) {
                RCC->APBENR2 |= RCC_APBENR2_TIM1EN;
            }
            else if constexpr(N == 2) {
                RCC->APBENR1 |= RCC_APBENR1_TIM2EN;
            }
            else if constexpr(N == 3) {
                RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
            }
            else {
                static_assert(false);
            }

            pin_a::template dir<Mcu::Input>();
            pin_b::template dir<Mcu::Input>();
            pin_t::template dir<Mcu::Input>();

            pin_a::pullup();
            pin_b::pullup();
            pin_t::pullup();

            static constexpr uint8_t pina_af = Mcu::Stm::AlternateFunctions::mapper_v<pin_a, Encoder, Mcu::Stm::AlternateFunctions::CC<1>>;
            static constexpr uint8_t pinb_af = Mcu::Stm::AlternateFunctions::mapper_v<pin_b, Encoder, Mcu::Stm::AlternateFunctions::CC<2>>;

            pin_a::afunction(pina_af);
            pin_b::afunction(pinb_af);

            mcuTimer->CCMR1 = [] consteval {
                    uint32_t r = 0;
                    r |= (0b01 << TIM_CCMR1_CC1S_Pos);
                    r |= (0b01 << TIM_CCMR1_CC2S_Pos);
                    return r;
            }();
            mcuTimer->CCER = [] consteval {
                    uint32_t r = 0;
                    return r;
            }();
            mcuTimer->SMCR = [] consteval {
                    uint32_t r = 0;
                    r |= (0b011 << TIM_SMCR_SMS_Pos);
                    return r;
            }();
            mcuTimer->ARR = Config::max;
            mcuTimer->CR1 |= TIM_CR1_CEN;
        }
#endif
        static inline uint32_t value() {
            return mcuTimer->CNT;
        }
    };
}
