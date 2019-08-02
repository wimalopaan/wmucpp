/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <cstdint>

#include "../common/concepts.h"

namespace AVR {
    template<typename MCU = DefaultMcuType>
    class Sleep {
        inline static constexpr auto mcu_sleep = AVR::getBaseAddr<typename MCU::Sleep>;
    public:
        struct PowerDown;

        template<typename T>
        inline static void init() {
            if constexpr(std::is_same_v<T, PowerDown>) {
                mcu_sleep()->ctrla.template set<MCU::Sleep::CtrlA_t::enable | MCU::Sleep::CtrlA_t::power_down>();
            }
            else {
                static_assert(std::false_v<T>, "wrong sleep mode");
            }
        }
        inline static void down() {
            __asm__ __volatile__ ( "sleep" "\n\t" :: ); 
        }
    private:
    };
}
