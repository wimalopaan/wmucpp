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

#include "mcu/mcu_traits.h"
#include "concepts.h"

namespace Mcu::Arm {
    namespace Atomic {
        struct DisableInterruptsRestore {
            inline DisableInterruptsRestore() {
                __disable_irq();
            }
            inline ~DisableInterruptsRestore() {
                __set_PRIMASK(primask);
            }
        private:
            const uint32_t primask = __get_PRIMASK();
        };
        
        auto access(const auto f) {
            DisableInterruptsRestore di; 
            return f();
        }
    }
}


