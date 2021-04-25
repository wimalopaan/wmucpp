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
        struct TCA {
            template<typename T>
            using pp_t = AVR::Util::PrescalerPair<T>;
            
//            static constexpr const uint8_t count = 1;

            enum class CtrlA_t : uint8_t {
                clksel2 = TCA_SINGLE_CLKSEL2_bm,
                clksel1 = TCA_SINGLE_CLKSEL1_bm,
                clksel0 = TCA_SINGLE_CLKSEL0_bm,
                enable = TCA_SINGLE_ENABLE_bm,
            };
            static inline constexpr std::array<pp_t<CtrlA_t>, 8> prescalerValues {
                pp_t<CtrlA_t>{CtrlA_t((0x00 << 1)), 1},
                pp_t<CtrlA_t>{CtrlA_t((0x01 << 1)), 2},
                pp_t<CtrlA_t>{CtrlA_t((0x02 << 1)), 4},
                pp_t<CtrlA_t>{CtrlA_t((0x03 << 1)), 8},
                pp_t<CtrlA_t>{CtrlA_t((0x04 << 1)), 16},
                pp_t<CtrlA_t>{CtrlA_t((0x05 << 1)), 64},
                pp_t<CtrlA_t>{CtrlA_t((0x06 << 1)), 256},
                pp_t<CtrlA_t>{CtrlA_t((0x07 << 1)), 1024},
            };
            ControlRegister<TCA, CtrlA_t> ctrla;

            enum class CtrlB_t : uint8_t {
                cmp2en = TCA_SINGLE_CMP2EN_bm,
                cmp1en = TCA_SINGLE_CMP1EN_bm,
                cmp0en = TCA_SINGLE_CMP0EN_bm,
                alupd  = TCA_SINGLE_ALUPD_bm,
                wgm2   = TCA_SINGLE_WGMODE2_bm,
                wgm1   = TCA_SINGLE_WGMODE1_bm,
                wgm0   = TCA_SINGLE_WGMODE0_bm,
                pwm    = wgm0 | wgm1,
                
                scmp0en  = 0x01,
                scmp1en  = 0x02,
                scmp2en  = 0x04,
                scmp3en  = 0x10,
                scmp4en  = 0x20,
                scmp5en  = 0x40,
            };
            ControlRegister<TCA, CtrlB_t> ctrlb;
            
            enum class CtrlC_t : uint8_t {
                cov2 = TCA_SINGLE_CMP2OV_bm,
                cov1 = TCA_SINGLE_CMP1OV_bm,
                cov0 = TCA_SINGLE_CMP0OV_bm,
                
                s0ov = 0x01,
                s1ov = 0x02,
                s2ov = 0x04,
                s3ov = 0x10,
                s4ov = 0x20,
                s5ov = 0x40,
            };
            ControlRegister<TCA, CtrlC_t> ctrlc;
            
            enum class CtrlD_t : uint8_t {
                splitm = 0x01
            };
            ControlRegister<TCA, CtrlD_t> ctrld;
            
            enum class CtrlE_t : uint8_t {
            };
            FlagRegister<TCA, CtrlE_t, WriteOnly> ctrleclr;
            FlagRegister<TCA, CtrlE_t, WriteOnly> ctrleset;

            enum class CtrlF_t : uint8_t {
            };
            FlagRegister<TCA, CtrlF_t, WriteOnly> ctrlfclr;
            FlagRegister<TCA, CtrlF_t, WriteOnly> ctrlfset;
            
            volatile uint8_t padding1;
            
            enum class Evctrl_t : uint8_t {
            };
            ControlRegister<TCA, Evctrl_t> evctrl;
            
            enum class Intctrl_t : uint8_t {
                ovf = 0x01,
                cmp0 = (1 << 4),
                cmp1 = (1 << 5),
                cmp2 = (1 << 6),
            };
            ControlRegister<TCA, Intctrl_t> intctrl;
            
            enum class Intflags_t : uint8_t {
                cmp2 = TCA_SINGLE_CMP2_bm,
                cmp1 = TCA_SINGLE_CMP1_bm,
                cmp0 = TCA_SINGLE_CMP0_bm,
                ovf = TCA_SINGLE_OVF_bm
            };
            FlagRegister<TCA, Intflags_t, ReadWrite> intflags;
            
            volatile uint8_t padding2[2];
            
            enum class Dbgctrl_t : uint8_t {
            };
            ControlRegister<TCA, Dbgctrl_t> dbgctrl;

            DataRegister<TCA, ReadWrite, std::byte> temp;

            volatile uint8_t padding3[0x1f - 0x10 + 1];
            
            DataRegister<TCA, ReadWrite, uint16_t> cnt;

            volatile uint8_t padding4[0x25 - 0x22 + 1];
            
            DataRegister<TCA, ReadWrite, uint16_t> per;
            DataRegister<TCA, ReadWrite, uint16_t> cmp0;
            DataRegister<TCA, ReadWrite, uint16_t> cmp1;
            DataRegister<TCA, ReadWrite, uint16_t> cmp2;

            volatile uint8_t padding5[0x35 - 0x2e + 1];

            DataRegister<TCA, ReadWrite, uint16_t> perbuf;
            DataRegister<TCA, ReadWrite, uint16_t> cmp0buf;
            DataRegister<TCA, ReadWrite, uint16_t> cmp1buf;
            DataRegister<TCA, ReadWrite, uint16_t> cmp2buf;

            
            template<int N> struct Address;
        };

//        std::integral_constant<uint8_t, sizeof(TCA)>::_;
        static_assert(sizeof(TCA) == 0x3e);

    }
}
