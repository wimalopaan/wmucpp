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
#include <std/utility>

namespace AVR {
    namespace Series0 {
        struct Clock final {
            enum class MClkCtrlA_t : uint8_t {
                clkout = CLKCTRL_CLKOUT_bm,
                clksel1 = CLKCTRL_CLKSEL_1_bm,
                clksel0 = CLKCTRL_CLKSEL_0_bm
            };
            ControlRegister<Clock, MClkCtrlA_t> mclkctrla;

            enum class MClkCtrlB_t : uint8_t {
                pdiv3 = CLKCTRL_PDIV_3_bm,
                pdiv2 = CLKCTRL_PDIV_2_bm,
                pdiv1 = CLKCTRL_PDIV_1_bm,
                pdiv0 = CLKCTRL_PDIV_0_bm,
                pen = CLKCTRL_PEN_bm,
            };
            
            template<typename T>
            using pp_t = AVR::Util::PrescalerPair<T>;
            
            static inline constexpr std::array<pp_t<MClkCtrlB_t>, 12> prescalerValues {
                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x00 << 1) | 0x00), 1},
                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x00 << 1) | 0x01), 2},
                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x01 << 1) | 0x01), 4},
                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x02 << 1) | 0x01), 8},
                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x03 << 1) | 0x01), 16},
                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x04 << 1) | 0x01), 32},
                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x05 << 1) | 0x01), 64},
                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x08 << 1) | 0x01), 6},
                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x09 << 1) | 0x01), 10},
                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x0a << 1) | 0x01), 12},
                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x0b << 1) | 0x01), 24},
                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x0c << 1) | 0x01), 48},
            };
            ControlRegister<Clock, MClkCtrlB_t> mclkctrlb;

            enum class MClkLock_t : uint8_t {
                lock = CLKCTRL_LOCKEN_bm,
            };
            ControlRegister<Clock, MClkLock_t> mclklock;
            
            static inline constexpr uintptr_t address = 0x0060;
        };

    }
}
