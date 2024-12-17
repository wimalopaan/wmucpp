#pragma once

#include <type_traits>
#include <concepts>

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "units.h"
#include "concepts.h"
#include "components.h"
#include "dmas.h"

namespace Mcu::Stm {
    namespace Dma {
        using namespace Units::literals;

        template<uint8_t N, typename MCU = void> struct Controller;

        template<uint8_t N, typename MCU> requires((N >= 1) && (N <= 2))
        struct Controller<N, MCU> {
            using component_t = Mcu::Components::Dma<N>;
            static inline /*constexpr */ DMA_TypeDef* const mcuDma = reinterpret_cast<DMA_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Dma<N>>::value);

#ifdef STM32G4
            static inline void init() {
                if constexpr (N == 1) {
                    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
                }
                else if constexpr (N == 2) {
                    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
                }
                else {
                    static_assert(false);
                }
            }
#endif
#ifdef STM32G0
            static inline void init() {
                if constexpr (N == 1) {
                    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
                }
#ifdef STM32G0B1xx
                else if constexpr(N == 2) {
                    RCC->AHBENR |= RCC_AHBENR_DMA2EN;
                }
#endif
                else {
                    static_assert(false);
                }
            }
#endif
        };
    }
}

