#pragma once

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "units.h"
#include "concepts.h"

#include <type_traits>
#include <concepts>

namespace Mcu::Stm {
    using namespace Units::literals;

    template<auto Nom, auto Div>
    struct VRefDiv;

    namespace Comp {
        template<typename PosPin, typename Component, typename MCU = DefaultMcu>
        struct InpSel;

        template<Mcu::Stm::G4xx MCU>
        struct InpSel<Pin<GPIO<A, MCU>, 1, MCU>, Mcu::Components::Comparator<1>, MCU> {
            static inline constexpr uint8_t value = 0;
        };
        template<Mcu::Stm::G4xx MCU>
        struct InpSel<Pin<GPIO<B, MCU>, 1, MCU>, Mcu::Components::Comparator<1>, MCU> {
            static inline constexpr uint8_t value = 1;
        };

        template<typename Neg, typename Component, typename MCU = DefaultMcu>
        struct InnSel;

        template<Mcu::Stm::G4xx MCU>
        struct InnSel<VRefDiv<1, 4>, Mcu::Components::Comparator<1>, MCU> {
            static inline constexpr uint8_t value = 0;
        };
        template<Mcu::Stm::G4xx MCU>
        struct InnSel<VRefDiv<1, 2>, Mcu::Components::Comparator<1>, MCU> {
            static inline constexpr uint8_t value = 1;
        };
        template<Mcu::Stm::G4xx MCU>
        struct InnSel<VRefDiv<3, 4>, Mcu::Components::Comparator<1>, MCU> {
            static inline constexpr uint8_t value = 2;
        };
    }

    template<uint8_t N, typename PosPin, typename NegInput, typename Debug, typename MCU = DefaultMcu>
    struct Comparator {
        using component_t = Mcu::Components::Comparator<N>;
        static inline /*constexpr */ COMP_TypeDef* const mcuComp = reinterpret_cast<COMP_TypeDef*>(Mcu::Stm::Address<component_t>::value);

        static inline void init() {
            constexpr uint8_t vinp = Comp::InpSel<PosPin, component_t>::value;
            // std::integral_constant<uint8_t, vinp>::_;
            constexpr uint8_t vinn = Comp::InnSel<NegInput, component_t>::value;
            // std::integral_constant<uint8_t, vinn>::_;

            mcuComp->CSR |= COMP_CSR_SCALEN | COMP_CSR_BRGEN;
            mcuComp->CSR |= (0b111) << COMP_CSR_HYST_Pos;
            mcuComp->CSR |= vinn << COMP_CSR_INMSEL_Pos;
            mcuComp->CSR |= vinp << COMP_CSR_INPSEL_Pos;
            mcuComp->CSR |= COMP_CSR_EN;
        }
        static inline void enableInt() {
            if constexpr(N == 1) {
                EXTI->IMR1 |= EXTI_IMR1_IM21;
                EXTI->RTSR1 |= EXTI_RTSR1_RT21;
            }
            else {
                static_assert(false);
            }
        }
        static inline void ratePeriodic() {
            // __disable_irq();
            if (mIdleCount == 0) {
                if (mPulseCount > 800) { // greater 0° Celsius
                    mSavedPulseCount = mPulseCount;
                    mPulseCount = 0;
                }
            }
            else {
                mIdleCount = mIdleCount - 1;
            }
            // __enable_irq();
        }
        static inline void isr() {
            mPulseCount = mPulseCount + 1;
            mIdleCount = 10;
        }
        static inline float temperatur() {
            return ((mSavedPulseCount * 256.0f) / 4096.0f) - 50.0f;
        }
        private:
        static inline uint32_t mSavedPulseCount{};
        static inline volatile uint32_t mPulseCount{};
        static inline volatile uint8_t mIdleCount{};
    };

    template<>
    struct Address<Mcu::Components::Comparator<1>> {
        static inline constexpr uintptr_t value = COMP1_BASE;
    };
}
