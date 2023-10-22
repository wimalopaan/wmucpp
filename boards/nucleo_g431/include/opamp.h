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
    struct Follower {
        static inline /*constexpr */ OPAMP_TypeDef* const mcuOpamp = reinterpret_cast<OPAMP_TypeDef*>(Mcu::Stm::Address<Follower<N, MCU>>::value);
        static inline void init() {
            mcuOpamp->CSR &= ~OPAMP_CSR_OPAMPINTEN;
            mcuOpamp->CSR |= (OPAMP_CSR_VMSEL_1 | OPAMP_CSR_VMSEL_0); // Follower
            mcuOpamp->CSR |= (OPAMP_CSR_VPSEL_1 | OPAMP_CSR_VPSEL_0); // DAC3 O1 / O2
            mcuOpamp->CSR |= OPAMP_CSR_OPAMPxEN;
        }        
    };    
    
    template<uint8_t N, typename MCU = DefaultMcu>
    struct PGA {
        static inline /*constexpr */ OPAMP_TypeDef* const mcuOpamp = reinterpret_cast<OPAMP_TypeDef*>(Mcu::Stm::Address<PGA<N, MCU>>::value);
        static inline void init() {
            mcuOpamp->CSR |= OPAMP_CSR_OPAMPINTEN;
            mcuOpamp->CSR |= OPAMP_CSR_VMSEL_1; // PGA
            mcuOpamp->CSR |= OPAMP_CSR_OPAMPxEN;
        }        
        static inline void gain(const uint8_t g) {
//            mcuOpamp->CSR &= ~OPAMP_CSR_OPAMPxEN;
            auto temp = mcuOpamp->CSR;
            temp &= ~OPAMP_CSR_PGGAIN_Msk;
            temp |= (g << OPAMP_CSR_PGGAIN_Pos);
            mcuOpamp->CSR = temp;
//            mcuOpamp->CSR |= OPAMP_CSR_OPAMPxEN;
        }
    };

    template<G4xx MCU>
    struct Address<PGA<1, MCU>> {
        static inline constexpr uintptr_t value = OPAMP1_BASE;        
    };
    template<G4xx MCU>
    struct Address<PGA<2, MCU>> {
        static inline constexpr uintptr_t value = OPAMP2_BASE;        
    };
    template<G4xx MCU>
    struct Address<PGA<3, MCU>> {
        static inline constexpr uintptr_t value = OPAMP3_BASE;        
    };
    template<G4xx MCU>
    struct Address<Follower<1, MCU>> {
        static inline constexpr uintptr_t value = OPAMP1_BASE;        
    };
    template<G4xx MCU>
    struct Address<Follower<3, MCU>> {
        static inline constexpr uintptr_t value = OPAMP3_BASE;        
    };
}
