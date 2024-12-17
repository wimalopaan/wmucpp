#pragma once

namespace Mcu::Stm {
    template<>
    struct Address<Mcu::Components::Dma<1>> {
        static inline constexpr uintptr_t value = DMA1_BASE;
    };
#ifdef DMA2_BASE
    template<>
    struct Address<Mcu::Components::Dma<2>> {
        static inline constexpr uintptr_t value = DMA2_BASE;
    };
#endif
}

#if defined(STM32G030xx) // DMA1: 5 Channels
# include "dmas_5.h"
#endif
#if defined(STM32G031xx) // DMA1: 5 Channels
# include "dmas_5.h"
#endif
#if defined(STM32G0B1xx) // DMA1: 7 Channels
# include "dmas_7.h"
#endif
#if defined(STM32G431xx) // DMA1: 6 Channels
# include "dmas_6.h"
#endif
#if defined(STM32G473xx) // DMA1: 8 Channels
# include "dmas_8.h"
#endif

