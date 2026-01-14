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
// #include "units.h"
// #include "concepts.h"

#include <type_traits>
#include <concepts>

#include "components.h"

namespace Mcu {
    namespace Stm {
        template<>
        struct Address<Mcu::Components::Rcc> {
            static inline RCC_TypeDef* const value = reinterpret_cast<RCC_TypeDef*>(RCC_BASE);
        };

    }
}
