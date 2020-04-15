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
        struct TCB {
            enum class CtrlA_t : uint8_t {
                standby = (1 << 6),
                syncupd = (1 << 4),
                clkdiv1 = ((0x00) << 1),
                clkdiv2 = ((0x01) << 1),
                clktca  = ((0x02) << 1),
                enable  = (1 << 0),
            };
            ControlRegister<TCB, CtrlA_t> ctrla;

            enum class CtrlB_t : uint8_t {
                async    = (1 << 6),
                ccmpinit = (1 << 5),
                ccmpen   = (1 << 4),
                mode_int = ((0x00) << 0),
                mode_to   = ((0x01) << 0),
                mode_capt = ((0x02) << 0),
                mode_frq = ((0x03) << 0),
                mode_pw = ((0x04) << 0),
                mode_frqpw = ((0x05) << 0),
                mode_single = ((0x06) << 0),
                mode_pwm8  = ((0x07) << 0),
            };
            ControlRegister<TCB, CtrlB_t> ctrlb;

            volatile std::byte reserved1;
            volatile std::byte reserved2;

            enum class EvCtrl_t : uint8_t {
                filter = (1 << 6),
                edge   = (1 << 4),
                captei = (1 << 0),
            };
            ControlRegister<TCB, EvCtrl_t> evctrl;

            enum class IntCtrl_t : uint8_t {
                capt = (1 << 0),
            };
            ControlRegister<TCB, IntCtrl_t> intctrl;

            enum class IntFlags_t : uint8_t {
                capt = (1 << 0),
            };
            FlagRegister<TCB, IntFlags_t> intflags;
            
            enum class Status_t : uint8_t {
                run = (1 << 0),
            };
            FlagRegister<TCB, Status_t, ReadOnly> status;

            enum class DbgCtrl_t : uint8_t {
                dbgrun = (1 << 0),
            };
            ControlRegister<TCB, DbgCtrl_t> dbgctrl;
            
            DataRegister<TCB, ReadOnly, std::byte> temp;
        
            DataRegister<TCB, ReadWrite, uint16_t> cnt;
            DataRegister<TCB, ReadWrite, uint16_t> ccmp;

            template<int N> struct Address;
        };
        
        static_assert(sizeof(TCB) == 0x0e);
    }
}
