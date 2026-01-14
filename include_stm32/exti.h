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
