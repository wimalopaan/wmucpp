#pragma once

#include "mcu/mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu/mcu_traits.h"

#include <type_traits>
#include <concepts>

namespace Mcu::Stm {
    using namespace Units::literals;

#ifdef USE_MCU_STM_V3
inline
#endif
    namespace V3 {
template<uint8_t N, typename MCU = void>
struct Dac {
    static inline /*constexpr */ DAC_TypeDef* const mcuDac = reinterpret_cast<DAC_TypeDef*>(Mcu::Stm::Address<Dac<N, MCU>>::value);
    static inline void init() {
        if constexpr(N == 1) {
            RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;
            mcuDac->CR |= DAC_CR_EN1;
            mcuDac->CR |= DAC_CR_EN2;
        }
        else if constexpr(N == 3) {
            RCC->AHB2ENR |= RCC_AHB2ENR_DAC3EN;
            mcuDac->MCR |= DAC_MCR_MODE1_0 | DAC_MCR_MODE1_1;
            mcuDac->MCR |= DAC_MCR_MODE2_0 | DAC_MCR_MODE2_1;
            mcuDac->CR |= DAC_CR_EN1;
            mcuDac->CR |= DAC_CR_EN2;
        }
    }
    static inline void set(uint16_t v) {
        mcuDac->DHR12R1 = v;
    }
    static inline void set2(uint16_t v) {
        mcuDac->DHR12R2 = v;
    }
};

}

#ifdef USE_MCU_STM_V2
    inline 
#endif
    namespace V2 {
        template<uint8_t N, typename MCU = void>
        struct Dac {
            static inline /*constexpr */ DAC_TypeDef* const mcuDac = reinterpret_cast<DAC_TypeDef*>(Mcu::Stm::Address<Dac<N, MCU>>::value);
            static inline void init() {
                if constexpr(N == 1) {
                    RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;
                    mcuDac->CR |= DAC_CR_EN1;
                    mcuDac->CR |= DAC_CR_EN2;
                }
                else if constexpr(N == 3) {
                    RCC->AHB2ENR |= RCC_AHB2ENR_DAC3EN;
                    mcuDac->MCR |= DAC_MCR_MODE1_0 | DAC_MCR_MODE1_1; 
                    mcuDac->MCR |= DAC_MCR_MODE2_0 | DAC_MCR_MODE2_1; 
                    mcuDac->CR |= DAC_CR_EN1;
                    mcuDac->CR |= DAC_CR_EN2;
                }
            }        
            static inline void set(uint16_t v) {
                mcuDac->DHR12R1 = v;
            }
            static inline void set2(uint16_t v) {
                mcuDac->DHR12R2 = v;
            }
        };
        
    }
    
#ifdef USE_MCU_STM_V1
    inline 
#endif
    namespace V1 {
        template<uint8_t N, typename MCU = void>
        struct Dac {
            static inline /*constexpr */ DAC_TypeDef* const mcuDac = reinterpret_cast<DAC_TypeDef*>(Mcu::Stm::Address<Dac<N, MCU>>::value);
            static inline void init() {
                if constexpr(N == 1) {
                    RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;
                    mcuDac->CR |= DAC_CR_EN1;
                }
                if constexpr(N == 3) {
                    RCC->AHB2ENR |= RCC_AHB2ENR_DAC3EN;
                    mcuDac->MCR |= DAC_MCR_MODE1_0 | DAC_MCR_MODE1_1; 
                    mcuDac->CR |= DAC_CR_EN1;
                    mcuDac->CR |= DAC_CR_EN2;
                }
            }        
            static inline void set(uint16_t v) {
                mcuDac->DHR12R1 = v;
            }
            static inline void set2(uint16_t v) {
                mcuDac->DHR12R2 = v;
            }
        };
    }
    
    template<G4xx MCU>
    struct Address<Dac<1, MCU>> {
        static inline constexpr uintptr_t value = DAC1_BASE;        
    };
    template<G4xx MCU>
    struct Address<Dac<3, MCU>> {
        static inline constexpr uintptr_t value = DAC3_BASE;        
    };
}
