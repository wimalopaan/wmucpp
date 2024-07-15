#pragma once

#include "mcu/mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu/mcu_traits.h"

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
            if (g == 0) {
                MODIFY_REG(mcuOpamp->CSR, OPAMP_CSR_VMSEL_Msk, (0b11 << OPAMP_CSR_VMSEL_Pos)); // follower
            }
            else {
                // MODIFY_REG(mcuOpamp->CSR, OPAMP_CSR_VMSEL_Msk, (0b10 << OPAMP_CSR_VMSEL_Pos)); // pga
                // MODIFY_REG(mcuOpamp->CSR, OPAMP_CSR_PGGAIN_Msk, ((g - 1) << OPAMP_CSR_PGGAIN_Pos)); // gain
                MODIFY_REG(mcuOpamp->CSR, (OPAMP_CSR_VMSEL_Msk | OPAMP_CSR_PGGAIN_Msk), ((0b10 << OPAMP_CSR_VMSEL_Pos) | ((g - 1) << OPAMP_CSR_PGGAIN_Pos))); // pga
                // auto temp = mcuOpamp->CSR;
                // temp &= ~OPAMP_CSR_PGGAIN_Msk;
                // temp |= ((g - 1) << OPAMP_CSR_PGGAIN_Pos);
                // mcuOpamp->CSR = temp;
            }
        }
        static inline void input(const uint8_t i) {
            MODIFY_REG(mcuOpamp->CSR, OPAMP_CSR_VPSEL_Msk, ((i & 0b11) << OPAMP_CSR_VPSEL_Pos));
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
