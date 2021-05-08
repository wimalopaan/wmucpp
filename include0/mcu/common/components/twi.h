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
        struct Twi final {
            enum class CtrlA_t : uint8_t {
                inputLevel = (0x01 << 6),
                sdaSetup   = (0x01 << 4),
                sdaHoldOff   = (0x00 << 2),
                sdaHold50ns  = (0x01 << 2),
                sdaHold300ns = (0x02 << 2),
                sdaHold500ns = (0x03 << 2),
                fmpEnable    = (0x01 << 1),
            };
            ControlRegister<Twi, CtrlA_t> ctrla;

            enum class DualCtrl_t : uint8_t {
                inputLevel = (0x01 << 6),
                sdaHoldOff   = (0x00 << 2),
                sdaHold50ns  = (0x01 << 2),
                sdaHold300ns = (0x02 << 2),
                sdaHold500ns = (0x03 << 2),
                fmpEnable    = (0x01 << 1),
                enable       = (0x01 << 0),
            };
            ControlRegister<Twi, DualCtrl_t> dualctrl;
            
            enum class DbgCtrl_t : uint8_t {
                dbgrun= (0x01 << 0),
            };
            ControlRegister<Twi, DbgCtrl_t> dbgctrl;

            enum class MCtrlA_t : uint8_t {
                readIEn = (0x01 << 7),
                writeIEn = (0x01 << 6),
                qCEn = (0x01 << 4),
                timeoutDisabled = (0x00 << 2),
                timeout50us = (0x01 << 2),
                timeout100us = (0x02 << 2),
                timeout200us = (0x03 << 2),
                sMEn = (0x01 << 1),
                enable = (0x01 << 0),
            };
            ControlRegister<Twi, MCtrlA_t> mctrla;

            enum class MCtrlB_t : uint8_t {
                flush  = (0x01 << 3),
                ackact = (0x01 << 2),
                MNoAct = (0x00 << 0),
                MRepStart = (0x01 << 0),
                MRecvTrans = (0x02 << 0),
                MStop = (0x03 << 0),
            };
            ControlRegister<Twi, MCtrlB_t> mctrlb;

            enum class MStatus_t : uint8_t {
                readIF = (0x01 << 7),
                writeIF = (0x01 << 6),
                clkHold = (0x01 << 5),
                rxAck   = (0x01 << 4),
                arbLost = (0x01 << 3),
                busErr = (0x01 << 2),
                busStateUnknown = (0x00 << 0),
                busStateIdle    = (0x01 << 0),
                busStateOwner   = (0x02 << 0),
                busStateBusy    = (0x03 << 0),
            };
            FlagRegister<Twi, MStatus_t> mstatus;
            
            DataRegister<Twi, ReadWrite, uint8_t> masterBaudrate;

            DataRegister<Twi, ReadWrite, std::byte> masterAddress;
            
            DataRegister<Twi, ReadWrite, std::byte> masterData;

            enum class SCtrlA_t : uint8_t {
                dataIEn = (0x01 << 7),
                // ...
            };
            ControlRegister<Twi, SCtrlA_t> sctrla;

            enum class SCtrlB_t : uint8_t {
                ackAct = (0x01 << 2),
                // ...
            };
            ControlRegister<Twi, SCtrlB_t> sctrlb;
            
            enum class SStatus_t : uint8_t {
                dataIF = (0x01 << 7),
                // ...
            };
            ControlRegister<Twi, SStatus_t> sstatus;

            DataRegister<Twi, ReadWrite, uint8_t> slaveAddress;

            DataRegister<Twi, ReadWrite> slaveData;

            DataRegister<Twi, ReadWrite> slaveAddressMask;
            
            template<int N> struct Address;
        };
        
        static_assert(sizeof(Twi) == 0x0f);
    }
}
