/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "mcu/mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu/mcu_traits.h"

#include <type_traits>
#include <concepts>

namespace Mcu::Stm {
    using namespace Units::literals;

	namespace V2 {
		template<uint8_t N, uint8_t INP, typename MCU = DefaultMcu>
		struct Follower {
			static inline /*constexpr */ OPAMP_TypeDef* const mcuOpamp = reinterpret_cast<OPAMP_TypeDef*>(Mcu::Stm::Address<Follower<N, INP, MCU>>::value);
			static inline void init() {
				mcuOpamp->CSR |= (OPAMP_CSR_VMSEL_1 | OPAMP_CSR_VMSEL_0); // Follower
				if constexpr(INP == 1) {
					mcuOpamp->CSR |= (0b01 << OPAMP_CSR_VPSEL_Pos); 
				}
				else if constexpr(INP == 2) {
					mcuOpamp->CSR |= (0b10 << OPAMP_CSR_VPSEL_Pos); 					
				}
				else {
					static_assert(false);
				}
				// mcuOpamp->CSR |= (OPAMP_CSR_VPSEL_1 | OPAMP_CSR_VPSEL_0); 
				// mcuOpamp->CSR |= OPAMP_CSR_FORCEVP; // test
				mcuOpamp->CSR |= OPAMP_CSR_OPAMPINTEN;
				mcuOpamp->CSR |= OPAMP_CSR_OPAMPxEN;
			}        
		};    
		
	}
	
    template<uint8_t N, typename MCU = void>
    struct Follower {
        static inline /*constexpr */ OPAMP_TypeDef* const mcuOpamp = reinterpret_cast<OPAMP_TypeDef*>(Mcu::Stm::Address<Follower<N, MCU>>::value);
        static inline void init() {
            mcuOpamp->CSR &= ~OPAMP_CSR_OPAMPINTEN;
            mcuOpamp->CSR |= (OPAMP_CSR_VMSEL_1 | OPAMP_CSR_VMSEL_0); // Follower
            mcuOpamp->CSR |= (OPAMP_CSR_VPSEL_1 | OPAMP_CSR_VPSEL_0); // DAC3 O1 / O2
            // mcuOpamp->CSR |= OPAMP_CSR_FORCEVP; // test
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
                MODIFY_REG(mcuOpamp->CSR, (OPAMP_CSR_VMSEL_Msk | OPAMP_CSR_PGGAIN_Msk), ((0b10 << OPAMP_CSR_VMSEL_Pos) | (((g - 1) | (mUseOffset ? 0b01000 : 0b00000)) << OPAMP_CSR_PGGAIN_Pos))); // pga
            }
        }
        static inline void input(const uint8_t i) {
            MODIFY_REG(mcuOpamp->CSR, OPAMP_CSR_VPSEL_Msk, ((i & 0b11) << OPAMP_CSR_VPSEL_Pos));
        }

#define OPAMP_CSR_OFFSET_Pos         (17U)
#define OPAMP_CSR_OFFSET_Msk         (0b11UL << OPAMP_CSR_OFFSET_Pos)

        static inline void useOffset(const bool use) {
            mUseOffset = use;
            MODIFY_REG(mcuOpamp->CSR, OPAMP_CSR_OFFSET_Msk, (mUseOffset ? 0b01 : 0b00) << OPAMP_CSR_OFFSET_Pos);
        }
        private:
        static inline bool mUseOffset{false};
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
	template<uint8_t INP, G4xx MCU>
    struct Address<V2::Follower<1, INP, MCU>> {
        static inline constexpr uintptr_t value = OPAMP1_BASE;        
    };
	template<uint8_t INP, G4xx MCU>
    struct Address<V2::Follower<2, INP, MCU>> {
        static inline constexpr uintptr_t value = OPAMP2_BASE;        
    };
	template<uint8_t INP, G4xx MCU>
    struct Address<V2::Follower<3, INP, MCU>> {
        static inline constexpr uintptr_t value = OPAMP3_BASE;        
    };
}
