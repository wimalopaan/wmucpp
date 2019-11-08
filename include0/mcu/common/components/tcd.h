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
    namespace Series1 {
        struct TCD {
            enum class CtrlA1_t : uint8_t {
                osc20m = (0x00 << 5),
                external = (0x02 << 5),
                system = (0x03 << 5),
            };
            enum class CtrlA2_t : uint8_t {
                div1 = (0x00 << 3),
                div4 = (0x01 << 3),
                div32 = (0x02 << 3),
            };
            enum class CtrlA3_t : uint8_t {
                syncPre1 = (0x00 << 1),
                syncPre2 = (0x01 << 1),
                syncPre4 = (0x02 << 1),
                syncPre8 = (0x03 << 1),
            };
            enum class CtrlA4_t : uint8_t {
                enable = (0x01 << 0),
            };
            ControlRegister<TCD, Meta::List<CtrlA1_t, CtrlA2_t, CtrlA3_t, CtrlA4_t>> ctrla;

            enum class CtrlB_t : uint8_t {
                oneRamp = (0x00 << 0),
                twoRamp = (0x01 << 0),
                fourRamp = (0x02 << 0),
                ds       = (0x03 << 0),
            };
            ControlRegister<TCD, CtrlB_t> ctrlb;

            enum class CtrlC_t : uint8_t {
                cmpDsel = (1 << 7),
                cmpCsel = (1 << 6),
                fifty = (1 << 3),
                aUpdate = (1 << 1),
                cmpOvr = (1 << 0),
            };
            ControlRegister<TCD, CtrlC_t> ctrlc;
            
            enum class CtrlD_t : uint8_t {
                B_Bon  = (1 << 7),
                B_Boff = (1 << 6),
                B_Aon  = (1 << 5),
                B_Aoff = (1 << 4),
                A_Bon  = (1 << 3),
                A_Boff = (1 << 2),
                A_Aon  = (1 << 1),
                A_Aoff = (1 << 0),
            };
            ControlRegister<TCD, CtrlC_t> ctrld;

            enum class CtrlE_t : uint8_t {
                diseoc = (1 << 7),
                scaptureb = (1 << 4),
                scapturea = (1 << 3),
                restart = (1 << 2),
                sync = (1 << 1),
                synceoc = (1 << 0),
            };
            ControlRegister<TCD, CtrlE_t> ctrle;

            volatile const std::byte reserved5;
            volatile const std::byte reserved6;
            volatile const std::byte reserved7;
            
            enum class EvCtrl1_t : uint8_t {
                neither  = (0x00 << 6),
                filteron = (0x01 << 6),
                asyncon  = (0x02 << 6),
            };
            enum class EvCtrl2_t : uint8_t {
                edge   = (1 << 4),
                action = (1 << 2),
                trigei = (1 << 0),
            };
            ControlRegister<TCD, Meta::List<EvCtrl1_t, EvCtrl2_t>> evctrla;
            ControlRegister<TCD, Meta::List<EvCtrl1_t, EvCtrl2_t>> evctrlb;

            volatile const std::byte reserveda;
            volatile const std::byte reservedb;
            
            enum class IntCtrl_t : uint8_t {
                trigb = (1 << 3),
                triga = (1 << 2),
                ovf   = (1 << 0),
            };
            ControlRegister<TCD, IntCtrl_t> intctrl;
            
            enum class IntFlags_t : uint8_t {
                trigb = (1 << 3),
                triga = (1 << 2),
                ovf   = (1 << 0),
            };
            FlagRegister<TCD, IntFlags_t> intflags;

            enum class Status_t : uint8_t {
                pwmactb = (1 << 7),
                pwmacta = (1 << 6),
                cmdready = (1 << 1),
                enready = (1 << 0),
            };
            FlagRegister<TCD, Status_t, ReadOnly> status;
            
            volatile const std::byte reservedf;

            enum class InputCtrl_t : uint8_t {
                none = 0x00,
                jmpwait = 0x01,
                execwait = 0x02,
                execfault = 0x03,
                freq = 0x04,
                execdt = 0x05,
                wait = 0x06,
                waitsw = 0x07,
                edgetrig = 0x08,
                edgetrigfreq = 0x09,
                lvltrigfreq = 0x0a,
            };
            ControlRegister<TCD, InputCtrl_t> inputctrla;
            ControlRegister<TCD, InputCtrl_t> inputctrlb;

            enum class FaultCtrl_t : uint8_t {
                cmpden = (1 << 7),
                cmpcen = (1 << 6),
                cmpben = (1 << 5),
                cmpaen = (1 << 4),
                cmpd = (1 << 3),
                cmpc = (1 << 2),
                cmpb = (1 << 1),
                cmpa = (1 << 0),
            };
            ControlRegister<TCD, FaultCtrl_t> faultctrl;

            volatile const std::byte reserved13;

            enum class DelayCtrl1_t : uint8_t {
                div1 = 0x00,
                div2 = 0x01,
                div4 = 0x02,
                div8 = 0x03,
            };
            enum class DelayCtrl2_t : uint8_t {
                cmpaset = 0x00,
                cmpaclr = 0x01,
                cmpbset = 0x02,
                cmpbclr = 0x03,
            };
            enum class DelayCtrl3_t : uint8_t {
                unused = 0x00,
                inputblank = 0x01,
                eventdelay = 0x02,
            };
            ControlRegister<TCD, Meta::List<DelayCtrl1_t, DelayCtrl2_t, DelayCtrl3_t>> delayctrl;
            
            DataRegister<TCD, ReadWrite, std::byte> delayValue;
            
            volatile const std::byte reserved16;
            volatile const std::byte reserved17;
            
            enum class DitCtrl_t : uint8_t {
            };
            ControlRegister<TCD, DitCtrl_t> ditctrl;

            DataRegister<TCD, ReadWrite, std::byte> ditValue;
 
            volatile const std::byte reserved1a;
            volatile const std::byte reserved1b;
            volatile const std::byte reserved1c;
            volatile const std::byte reserved1d;
            
            enum class DebugCtrl_t : uint8_t {
            };
            ControlRegister<TCD, DebugCtrl_t> debugctrl;

            volatile const std::byte reserved1f;
            volatile const std::byte reserved20;
            volatile const std::byte reserved21;
            
            DataRegister<TCD, ReadWrite, uint16_t> capturea;
            DataRegister<TCD, ReadWrite, uint16_t> captureb;
            
            volatile const std::byte reserved26;
            volatile const std::byte reserved27;
            
            DataRegister<TCD, ReadWrite, uint16_t> cmpaset;
            DataRegister<TCD, ReadWrite, uint16_t> cmpaclr;

            DataRegister<TCD, ReadWrite, uint16_t> cmpbset;
            DataRegister<TCD, ReadWrite, uint16_t> cmpbclr;
            
            template<int N> struct Address;
        };
        
        static_assert(sizeof(TCD) == 0x30);
    }
}
