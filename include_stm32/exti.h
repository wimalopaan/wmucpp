#pragma once

#include "mcu/mcu.h"
#include "timer.h"
#include "units.h"
#include "concepts.h"
#include "mcu/mcu_traits.h"
#include "meta.h"
#include "mcu/alternate.h"
#include "etl/ranged.h"

#include <type_traits>
#include <concepts>
#include <algorithm>
#include <array>
#include <numeric>


namespace Mcu::Stm {
    using namespace Units::literals;

    template<typename PIN, typename MUC = DefaultMcu>
    struct ExtInt {
        static inline void isr() {
            ++mIsrCount;
        }
        static inline void init() {
            SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PB;
            EXTI->RTSR1 |= EXTI_RTSR1_RT6;
            EXTI->IMR1 |= EXTI_IMR1_IM6;
        }
    private:
        static inline uint16_t mIsrCount{};
    public:
        static inline volatile const auto& isrCount{mIsrCount};
    };
    
}
