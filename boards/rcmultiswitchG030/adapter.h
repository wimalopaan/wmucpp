/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include <type_traits>
#include <output.h>
#include <mcu/alternate.h>

#include "tick.h"

namespace Local {
    template<typename Pwm, auto Channel, bool Negativ = false, bool HasOutput = true, typename Debug = void>
    struct PwmAdapter {
        static inline constexpr bool hasOutput = HasOutput;
        static inline constexpr uint8_t channel = Channel;
        using pwm = Pwm;
        using component_t = Pwm::component_t;
        using polarity_t = std::conditional_t<Negativ, Mcu::Stm::AlternateFunctions::Negativ, Mcu::Stm::AlternateFunctions::Positiv>;

        static inline void duty(const uint8_t d) {
            const uint16_t dv = (Pwm::period * (uint32_t)d) / 100;
            if constexpr(Channel == 1) {
                IO::outl<Debug>("duty1: ", dv);
                Pwm::duty1(dv);
            }
            else if constexpr(Channel == 2){
                IO::outl<Debug>("duty2: ", dv);
                Pwm::duty2(dv);
            }
            else if constexpr(Channel == 3){
                IO::outl<Debug>("duty3: ", dv);
                Pwm::duty3(dv);
            }
            else if constexpr(Channel == 4){
                IO::outl<Debug>("duty4: ", dv);
                Pwm::duty4(dv);
            }
            else {
                static_assert(false);
            }
        }
    };

}

