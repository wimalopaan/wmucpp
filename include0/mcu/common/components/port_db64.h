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
        struct VPort final {
            DataRegister<VPort, ReadWrite, std::byte> dir;
            DataRegister<VPort, ReadWrite, std::byte> out;
            DataRegister<VPort, ReadWrite, std::byte> in;
            DataRegister<VPort, ReadWrite, std::byte> intflags;
            
            template<typename P> struct Address;
        };

        struct PortRegister final {
            DataRegister<PortRegister, ReadWrite, std::byte> dir;
            DataRegister<PortRegister, ReadWrite, std::byte> dirset;
            DataRegister<PortRegister, ReadWrite, std::byte> dirclr;
            DataRegister<PortRegister, ReadWrite, std::byte> dirtgl;
            DataRegister<PortRegister, ReadWrite, std::byte> out;
            DataRegister<PortRegister, ReadWrite, std::byte> outset;
            DataRegister<PortRegister, ReadWrite, std::byte> outclr;
            DataRegister<PortRegister, ReadWrite, std::byte> outtgl;
            DataRegister<PortRegister, ReadWrite, std::byte> in;
            DataRegister<PortRegister, ReadWrite, std::byte> intflags; // Flagregister
            
            enum class PortCtrl_t : uint8_t {
                srl = 0x01
            };
            ControlRegister<PortRegister, PortCtrl_t> portctrl;

            enum class PinCtrl_t : uint8_t {
                inven  = 0x80,
                inlvl = 0x40,
                pullup = 0x08,
                intdisable= 0x00,
                bothedges = 0x01,
                rising    = 0x02,
                falling   = 0x03,
                disable   = 0x04,
                level     = 0x05,
            };
            
            ControlRegister<PortRegister, PinCtrl_t> pinconfig;
            FlagRegister<PortRegister, std::byte, WriteOnly, std::byte> pinctrlupd;
            FlagRegister<PortRegister, std::byte, WriteOnly, std::byte> pinctrlset;
            FlagRegister<PortRegister, std::byte, WriteOnly, std::byte> pinctrlclr;
            
            const volatile std::byte reserved;
            
            std::array<ControlRegister<PortRegister, PinCtrl_t>, 8> pinctrl;            
            
            template<typename P> struct Address;
        };
//        std::integral_constant<uint8_t, sizeof(PortRegister)>::_;
        static_assert(sizeof(PortRegister) == 0x18);

    }
}


