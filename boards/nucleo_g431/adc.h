#pragma once

#include "mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu_traits.h"

#include <type_traits>
#include <concepts>

namespace Mcu::Stm {
    using namespace Units::literals;
    
    template<uint8_t N, typename MCU = void>
    struct Adc {
        static inline /*constexpr */ ADC_TypeDef* const mcuDac = reinterpret_cast<ADC_TypeDef*>(Mcu::Stm::Address<Adc<N, MCU>>::value);
        static inline void init() {
            
        }
    };
    
    template<G4xx MCU> 
    struct Address<Adc<1, MCU>> {
        static inline constexpr uintptr_t value = ADC1_BASE;
    };
    
}
