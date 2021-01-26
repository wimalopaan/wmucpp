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
    namespace SeriesDa {
        struct NvmCtrl final {
            enum class CtrlA_t: uint8_t {
                nocmd   = 0x00,
                noop    = 0x01,
                flwr    = 0x02,
                flper   = 0x08,
                flmper2   = 0x09,
                flmper4   = 0x0a,
                flmper8   = 0x0b,
                flmper16   = 0x0c,
                flmper32   = 0x0d,
                eewr   = 0x12,
                eeerwr   = 0x13,
                eeber   = 0x18,
                eember2   = 0x19,
                eember4   = 0x1a,
                eember8   = 0x1b,
                eember16   = 0x1c,
                eember32   = 0x1d,
                cher    = 0x20,
                eecher   = 0x30
            };
            ControlRegister<NvmCtrl, CtrlA_t> ctrla;
            
            enum class CtrlB_t: uint8_t {
            };
            ControlRegister<NvmCtrl, CtrlB_t> ctrlb;

            enum class Status_t: uint8_t {
                enone   = (0x00 << 4),
                einvalid   = (0x01 << 4),
                ewriteprotect= (0x02 << 4),
                ecmdcollision= (0x03 << 4),
                eebusy = 0x02,
                fbusy = 0x01,
            };
            FlagRegister<NvmCtrl, Status_t, ReadWrite> status;

            enum class IntCtrl_t: uint8_t {
                eeready   = (0x01 << 0)
            };
            FlagRegister<NvmCtrl, IntCtrl_t, ReadWrite> intctrl;

            enum class IntFlags_t: uint8_t {
                eeready   = (0x01 << 0)
            };
            FlagRegister<NvmCtrl, IntFlags_t, ReadWrite> intflags;
            
            const volatile std::byte res;
            
            DataRegister<NvmCtrl, ReadWrite, uint16_t> data;

            DataRegister<NvmCtrl, ReadWrite, uint8_t> addr0;
            DataRegister<NvmCtrl, ReadWrite, uint8_t> addr1;
            DataRegister<NvmCtrl, ReadWrite, uint8_t> addr2;
            
            static inline constexpr uintptr_t address = 0x1000;
        };
       static_assert(sizeof(NvmCtrl) == 11);

    }
}
#pragma once
