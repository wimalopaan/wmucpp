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
    struct Stm32G431;
    struct Stm32G473;
    struct Stm32G030;
    struct Stm32G031;
    struct Stm32G051;
    struct Stm32G0B1;
}
#if defined(STM32G431xx)
# include <stm32g4xx.h>
  using DefaultMcu = Mcu::Stm::Stm32G431;
#elif defined(STM32G473xx)
# include <stm32g4xx.h>
  using DefaultMcu = Mcu::Stm::Stm32G473;
#elif defined(STM32G030xx)
# include <stm32g0xx.h>
  using DefaultMcu = Mcu::Stm::Stm32G030;
#elif defined(STM32G031xx)
# include <stm32g0xx.h>
  using DefaultMcu = Mcu::Stm::Stm32G031;
#elif defined(STM32G051xx)
# include <stm32g0xx.h>
  using DefaultMcu = Mcu::Stm::Stm32G051;
#elif defined(STM32G0B1xx)
# include <stm32g0xx.h>
  using DefaultMcu = Mcu::Stm::Stm32G0B1;
#else
# error "no MCU defined" 
#endif



