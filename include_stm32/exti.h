#pragma once

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/alternate.h"

#include <type_traits>
#include <concepts>
#include <algorithm>
#include <array>
#include <numeric>

namespace Mcu::Stm {
    // using namespace Units::literals;

    namespace ExtI {
        template<typename Port> struct Map;
        template<> struct Map<A> {
            static inline constexpr uint8_t value = 0x00;
        };
        template<> struct Map<B> {
            static inline constexpr uint8_t value = 0x01;
        };
        template<> struct Map<C> {
            static inline constexpr uint8_t value = 0x02;
        };
        template<> struct Map<D> {
            static inline constexpr uint8_t value = 0x03;
        };
        template<> struct Map<E> {
            static inline constexpr uint8_t value = 0x04;
        };
        template<> struct Map<F> {
            static inline constexpr uint8_t value = 0x05;
        };
    }

    template<typename PIN, typename MUC = DefaultMcu>
    struct ExtInt {
        // static inline void init() {
        //     SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PB;
        //     EXTI->RTSR1 |= EXTI_RTSR1_RT6;
        //     EXTI->IMR1 |= EXTI_IMR1_IM6;
        // }
    private:
        static inline uint16_t mIsrCount{};
    public:
        static inline volatile const auto& isrCount{mIsrCount};
    };
    
}
