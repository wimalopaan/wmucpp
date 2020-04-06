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
    struct Reset;
    template<AVR::Concepts::At01Series MCU>
    struct Reset<MCU> {
        using flags_t = MCU::Reset::Flags_t;
        inline static constexpr auto mcu_reset = AVR::getBaseAddr<typename MCU::Reset>;
        
        inline static void onWatchDog(const auto f) {
            mcu_reset()->flags.template testAndReset<flags_t::wd>(f);
        }
        inline static void noWatchDog(const auto f) {
            if (!mcu_reset()->flags.template isSet<flags_t::wd>()) {
                f();
            }
        }
    };
}
