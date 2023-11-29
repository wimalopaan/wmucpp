#pragma once

#include "mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu_traits.h"

#include <type_traits>
#include <concepts>

namespace Mcu::Stm {
    using namespace Units::literals;

#ifdef USE_MCU_STM_V2
    inline 
#endif
    namespace V2 {
        template<uint8_t N, typename MCU = void>
        struct Spi {
            static inline /*constexpr */ SPI_TypeDef* const mcuSpi= reinterpret_cast<SPI_TypeDef*>(Mcu::Stm::Address<Spi<N, MCU>>::value);
            static inline void init() {
                if constexpr(N == 1) {
                    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
                }
                else if constexpr(N == 3) {
                }
                else {
                    static_assert(false);
                }
            }        
        };
    }
    
    
    template<G4xx MCU>
    struct Address<Spi<1, MCU>> {
        static inline constexpr uintptr_t value = DAC1_BASE;        
    };
}
