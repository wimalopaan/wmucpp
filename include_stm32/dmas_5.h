/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

namespace Mcu::Stm {
    template<>
    struct Address<Mcu::Components::DmaChannel<Mcu::Components::Dma<1>, 1>> {
        static inline constexpr uintptr_t value = DMA1_Channel1_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel0_BASE;
    };
    template<>
    struct Address<Mcu::Components::DmaChannel<Mcu::Components::Dma<1>, 2>> {
        static inline constexpr uintptr_t value = DMA1_Channel2_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel1_BASE;
    };
    template<>
    struct Address<Mcu::Components::DmaChannel<Mcu::Components::Dma<1>, 3>> {
        static inline constexpr uintptr_t value = DMA1_Channel3_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel2_BASE;
    };
    template<>
    struct Address<Mcu::Components::DmaChannel<Mcu::Components::Dma<1>, 4>> {
        static inline constexpr uintptr_t value = DMA1_Channel4_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel3_BASE;
    };
    template<>
    struct Address<Mcu::Components::DmaChannel<Mcu::Components::Dma<1>, 5>> {
        static inline constexpr uintptr_t value = DMA1_Channel5_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel4_BASE;
    };
}
