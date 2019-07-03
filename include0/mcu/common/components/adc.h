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

#include "vref.h"

namespace AVR {
    namespace Series0 {
        struct Adc {
            enum class CtrlA_t : uint8_t {
                standby = (1 << 7),
                ressel  = (1 << 2),
                freerun = (1 << 1),
                enable  = (1 << 0),
            };
            ControlRegister<Adc, CtrlA_t> ctrla;
            
            enum class CtrlB_t : uint8_t {
                accu_none = 0x00, 
                accu_2    = 0x01, 
                accu_4    = 0x02, 
                accu_8    = 0x03, 
                accu_16   = 0x04, 
                accu_32   = 0x05, 
                accu_64   = 0x06, 
            };
            ControlRegister<Adc, CtrlB_t> ctrlb;
            
            enum class CtrlC_t : uint8_t {
                samcap        = (1 << 6),
                ref_internal  = (0x00 << 4),
                ref_vdd       = (0x01 << 4),
                ref_vrefa     = (0x02 << 4),
                div2          = 0x00, 
                div4          = 0x01, 
                div8          = 0x02, 
                div16         = 0x03, 
                div32         = 0x04, 
                div64         = 0x05, 
                div128        = 0x06, 
                div256        = 0x07, 
            };
            ControlRegister<Adc, CtrlC_t> ctrlc;
            
            enum class CtrlD_t : uint8_t {
                delay0   = (0x00 << 5),
                delay16  = (0x01 << 5),
                delay32  = (0x02 << 5),
                delay64  = (0x03 << 5),
                delay128 = (0x04 << 5),
                delay256 = (0x05 << 5),
                asdv     = (1 << 4),
                
            };
            ControlRegister<Adc, CtrlD_t> ctrld;

            enum class CtrlE_t : uint8_t {
                none   = 0x00,
                below  = 0x01,
                above  = 0x02,
                inside = 0x03,
                outside= 0x04,
            };
            ControlRegister<Adc, CtrlE_t> ctrle;
            
            DataRegister<Adc, ReadWrite, std::byte> sampctl;

            enum class MuxPos_t : uint8_t {
                ain0    = 0x00,
                ain1    = 0x01,
                ain2    = 0x02,
                ain3    = 0x03,
                ain4    = 0x04,
                ain5    = 0x05,
                ain6    = 0x06,
                ain7    = 0x07,
                ain8    = 0x08,
                ain9    = 0x09,
                ain10   = 0x0a,
                ain11   = 0x0b,
                ain12   = 0x0c,
                ain13   = 0x0d,
                ain14   = 0x0e,
                ain15   = 0x0f,
                dacref0 = 0x1c,
                temp    = 0x1e,
                gnd     = 0x1f,
            };
            ControlRegister<Adc, MuxPos_t> muxpos;

            volatile std::byte reserved;
            
            enum class Command_t : uint8_t {
                stconv = 0x01,
            };
            ControlRegister<Adc, Command_t> command;

            enum class EvCtrl_t : uint8_t {
                startei = 0x01,
            };
            ControlRegister<Adc, EvCtrl_t> evctrl;
            
            enum class IntCtrl_t : uint8_t {
                resrdy = 0x01,
                wcomp  = 0x02,
            };
            ControlRegister<Adc, IntCtrl_t> intctrl;

            FlagRegister<Adc, IntCtrl_t> intflags;
            
            enum class DbgCtrl_t : uint8_t {
                dbgrun = 0x01,
            };
            ControlRegister<Adc, DbgCtrl_t> dbgctrl;

            DataRegister<Adc, ReadWrite, std::byte> temp;

            volatile std::byte reserved1;
            volatile std::byte reserved2;
            
            DataRegister<Adc, ReadWrite, uint16_t> res;
            DataRegister<Adc, ReadWrite, uint16_t> winlt;
            DataRegister<Adc, ReadWrite, uint16_t> winht;
            
            enum class Calib_t : uint8_t {
                dutycycle = 0x01,
            };
            ControlRegister<Adc, Calib_t> calib;

            static inline constexpr uintptr_t address = 0x0600;
        };
        static_assert(sizeof(Adc) == 0x17);
        
    }
}
