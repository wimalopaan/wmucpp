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
    
    template<uint8_t N, uint8_t Channel = 0, typename MCU = void> struct Dma;
    
    template<uint8_t N, uint8_t Channel, typename MCU>
    requires (N >= 1) && (N <=2)
    struct Dma<N, Channel, MCU> {
        static inline /*constexpr */ DMA_TypeDef* const mcuDma = reinterpret_cast<DMA_TypeDef*>(Mcu::Stm::Address<Dma<N, Channel, MCU>>::value);
        
        static inline void init() {
            if constexpr(N == 1) {
                RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
            }
            else if constexpr(N == 2) {
                RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
            }
            else {
                static_assert(false);
            }
        }

    };
    template<uint8_t Ch, G4xx MCU> 
    struct Address<Dma<1, Ch, MCU>> {
        static inline constexpr uintptr_t value = DMA1_BASE;
    };

//    template<G4xx MCU> 
//    struct Address<DmaMux<0, MCU>> {
//        static inline constexpr uintptr_t value = DMAMUX1_Channel0_BASE;
//    };
}
