/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
    struct Address<Mcu::Components::Dma<1>> {
        static inline constexpr uintptr_t value = DMA1_BASE;
    };
#ifdef DMA2_BASE
    template<>
    struct Address<Mcu::Components::Dma<2>> {
        static inline constexpr uintptr_t value = DMA2_BASE;
    };
#endif
}

#if defined(STM32G030xx) // DMA1: 5 Channels
# include "dmas_5.h"
#endif
#if defined(STM32G031xx) // DMA1: 5 Channels
# include "dmas_5.h"
#endif
#if defined(STM32G051xx) // DMA1: 7 Channels
# include "dmas_7.h"
#endif
#if defined(STM32G0B1xx) // DMA1: 7 Channels
# include "dmas_7.h"
#endif
#if defined(STM32G431xx) // DMA1: 6 Channels
# include "dmas_6.h"
#endif
#if defined(STM32G473xx) // DMA1: 8 Channels
# include "dmas_8.h"
#endif

