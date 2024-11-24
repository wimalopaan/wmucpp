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
template<uint8_t N, typename MCU = DefaultMcu>
struct Dac {
    static inline /*constexpr */ DAC_TypeDef* const mcuDac = reinterpret_cast<DAC_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Dac<N>>::value);

    // using value_type = uint16_t; // uint32_t???
    using value_type = uint32_t; // uint32_t???

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
        else {
            static_assert(false);
        }
    }

    template<uint8_t CH = 1>
    static inline uint8_t dmamuxSrc() {
        if constexpr(CH == 1) {
            return 6;
        }
        else if constexpr(CH == 2) {
            return 7;
        }
        else {
            static_assert(false);
        }
    }

    template<uint8_t CH = 1>
    static inline uint8_t dmaEnable(const uint8_t trigger) {
        if constexpr(CH == 1) {
            MODIFY_REG(mcuDac->CR, DAC_CR_TSEL1_Msk, trigger << DAC_CR_TSEL1_Pos);
            mcuDac->CR |= DAC_CR_DMAEN1;
            mcuDac->CR |= DAC_CR_TEN1;
            return 6;
        }
        else if constexpr(CH == 2) {
            MODIFY_REG(mcuDac->CR, DAC_CR_TSEL2_Msk, trigger << DAC_CR_TSEL2_Pos);
            mcuDac->CR |= DAC_CR_DMAEN2;
            mcuDac->CR |= DAC_CR_TEN2;
            return 7;
        }
        else {
            static_assert(false);
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
    
    template<>
    struct Address<Mcu::Components::Dac<1>> {
        static inline constexpr uintptr_t value = DAC1_BASE;        
    };
    template<>
    struct Address<Mcu::Components::Dac<3>> {
        static inline constexpr uintptr_t value = DAC3_BASE;        
    };
}
