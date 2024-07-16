#pragma once

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
// #include "units.h"
// #include "concepts.h"

#include <type_traits>
#include <concepts>

namespace Mcu {
    namespace Stm {
        template<>
        struct Address<Mcu::Components::Rcc> {
            static inline RCC_TypeDef* const value = reinterpret_cast<RCC_TypeDef*>(RCC_BASE);
        };

    }
}
