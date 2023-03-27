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
    namespace SeriesDb {
        struct Clock final {
            enum class MClkCtrlA_t : uint8_t {
                clkout = CLKCTRL_CLKOUT_bm,
//                clksel3 = CLKCTRL_CLKSEL_3_bm,
                clksel2 = CLKCTRL_CLKSEL_2_bm,
                clksel1 = CLKCTRL_CLKSEL_1_bm,
                clksel0 = CLKCTRL_CLKSEL_0_bm,
                oschf   = CLKCTRL_CLKSEL_OSCHF_gc,
                osc32k  = CLKCTRL_CLKSEL_OSC32K_gc,
                xosc32k = CLKCTRL_CLKSEL_XOSC32K_gc,
                extclk  = CLKCTRL_CLKSEL_EXTCLK_gc
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

            enum class MClkCtrlC_t : uint8_t {
                clkmain  = CLKCTRL_CFDSRC_CLKMAIN_gc,
                xoschf   = CLKCTRL_CFDSRC_XOSCHF_gc,
                xosc32k  = CLKCTRL_CFDSRC_XOSC32K_gc,
                cfdtst   = CLKCTRL_CFDEN_bm,
                cfden    = CLKCTRL_CFDEN_bm,
            };
            ControlRegister<Clock, MClkCtrlC_t> mclkctrlc;

            enum class MClkIntCtrl_t : uint8_t {
                nmi_int = CLKCTRL_INTTYPE_bm,
                reg_int = 0x00,
                cfd  = CLKCTRL_CFD_bm
            };
            ControlRegister<Clock, MClkIntCtrl_t> mclkintctrl;
            
            enum class MClkIntFlags_t : uint8_t {
                cfd = CLKCTRL_CFD_bm
            };
            ControlRegister<Clock, MClkIntFlags_t> mclkintflags;
            
            enum class MClkStatus_t : uint8_t {
            };
            FlagRegister<Clock, MClkStatus_t> mclkstatus;

            volatile const std::byte reserved0x06;
            volatile const std::byte reserved0x07;
            
            enum class OscHFCtrlA_t : uint8_t {
                f_1mhz = 0x00 << 2,
                f_2mhz = 0x01 << 2,
                f_3mhz = 0x02 << 2,
                f_4mhz = 0x03 << 2,
                f_8mhz = 0x05 << 2,
                f_12mhz = 0x06 << 2,
                f_16mhz = 0x07 << 2,
                f_20mhz = 0x08 << 2,
                f_24mhz = 0x09 << 2,
                f_28mhz = 0x0a << 2, // experimental
                f_32mhz = 0x0b << 2 // experimental
            };
            ControlRegister<Clock, OscHFCtrlA_t> oschfctrla;
            
            DataRegister<Clock, ReadWrite, std::byte> oschtune;

            volatile const std::byte reserved0x0a;
            volatile const std::byte reserved0x0b;
            volatile const std::byte reserved0x0c;
            volatile const std::byte reserved0x0d;
            volatile const std::byte reserved0x0e;
            volatile const std::byte reserved0x0f;
            
            enum class PllCtrlA_t : uint8_t {
            };
            ControlRegister<Clock, PllCtrlA_t> pllctrla;

            volatile const std::byte reserved0x11;
            volatile const std::byte reserved0x12;
            volatile const std::byte reserved0x13;
            volatile const std::byte reserved0x14;
            volatile const std::byte reserved0x15;
            volatile const std::byte reserved0x16;
            volatile const std::byte reserved0x17;
            
            enum class Osc32kCtrlA_t : uint8_t {
            };
            ControlRegister<Clock, Osc32kCtrlA_t> osc32kctrla;
            
            volatile const std::byte reserved0x19;
            volatile const std::byte reserved0x1a;
            volatile const std::byte reserved0x1b;
            
            enum class XOsc32kCtrlA_t : uint8_t {
            };
            ControlRegister<Clock, XOsc32kCtrlA_t> xosc32kctrla;
            
            volatile const std::byte reserved0x1d;
            volatile const std::byte reserved0x1e;
            volatile const std::byte reserved0x1f;
           
            enum class XOscHfCtrlA_t : uint8_t {
            };
            ControlRegister<Clock, XOscHfCtrlA_t> xoschfctrla;
            
            static inline constexpr uintptr_t address = 0x0060;
        };
        static_assert(sizeof(Clock) == 0x21);
    }
}
