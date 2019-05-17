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

#pragma pack(push)
#pragma pack(1)

namespace AVR {
    struct ATMega4809 final {
        template<typename T>
        inline static constexpr bool is_atomic() {return false;}
        
        ATMega4809() = delete;
        
        struct Ram final {
            inline static constexpr uintptr_t begin = RAMSTART;  
            inline static constexpr uintptr_t end   = RAMEND;  
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
            DataRegister<PortRegister, ReadWrite, std::byte> inflags;
            DataRegister<PortRegister, ReadWrite, std::byte> portctrl;

            volatile uint8_t padding[0x0f - 0x0b + 1];
            
            DataRegister<PortRegister, ReadWrite, std::byte> pin0ctrl;
            DataRegister<PortRegister, ReadWrite, std::byte> pin1ctrl;
            DataRegister<PortRegister, ReadWrite, std::byte> pin2ctrl;
            DataRegister<PortRegister, ReadWrite, std::byte> pin3ctrl;
            DataRegister<PortRegister, ReadWrite, std::byte> pin4ctrl;
            DataRegister<PortRegister, ReadWrite, std::byte> pin5ctrl;
            DataRegister<PortRegister, ReadWrite, std::byte> pin6ctrl;
            DataRegister<PortRegister, ReadWrite, std::byte> pin7ctrl;

            template<typename P> struct Address;
        };
//        std::integral_constant<uint8_t, sizeof(PortRegister)>::_;
        static_assert(sizeof(PortRegister) == 0x18);
        
        struct Usart {
            static constexpr const uint8_t count = 4;
            DataRegister<Usart, ReadOnly, std::byte> rxd;
            enum class RXDATAH : uint8_t {
                rxcif = USART_RXCIF_bm,
                bufovl = USART_BUFOVF_bm,
                ferr = USART_FERR_bm,
                perr = USART_PERR_bm,
                data8 = USART_DATA8_bm,
            };
            ControlRegister<Usart, RXDATAH> rxdh;
            DataRegister<Usart, ReadWrite, std::byte> txd;

            volatile uint8_t reserved2;
            template<int N> struct Address;
        };
        
    };
    template<>
    constexpr bool ATMega4809::is_atomic<uint8_t>() {return true;}
}

namespace std {
//    template<>
//    struct enable_bitmask_operators<AVR::ATMega328PB::Status::Bits> : std::true_type {};
}

namespace AVR {
    template<> struct ATMega4809::PortRegister::Address<A> {
        inline static constexpr uintptr_t value = 0x0400;
    };
    template<> struct ATMega4809::PortRegister::Address<B> {
        inline static constexpr uintptr_t value = 0x0420;
    };
    template<> struct ATMega4809::PortRegister::Address<C> {
        inline static constexpr uintptr_t value = 0x0440;
    };
    template<> struct ATMega4809::PortRegister::Address<D> {
        inline static constexpr uintptr_t value = 0x0460;
    };

}
#pragma pack(pop)
