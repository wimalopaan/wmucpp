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

#include "components2.h"

#pragma pack(push)
#pragma pack(1)

namespace AVR {
    struct ATTiny1624 final {
        
        template<typename T>
        inline static constexpr bool is_atomic() {return false;}
        
        ATTiny1624() = delete;
        
//        struct Ram final {
//            inline static constexpr uintptr_t begin = RAMSTART;  
//            inline static constexpr uintptr_t end   = RAMEND;  
//        };
        
        using Gpior = AVR::Series0::GPIOR;
        using Cpu = AVR::Series0::Cpu;
        using Clock = AVR::Series0::Clock;
        using Rtc = AVR::Series0::Rtc;
        using Usart = AVR::SeriesDa::Usart;
        using TCA = AVR::Series0::TCA;
        using TCB = AVR::Series0::TCB;
        using Sleep = AVR::Series0::Sleep;
        using Adc = AVR::Series2::Adc;
        using Vref = AVR::Series1::Vref;
        using Spi = AVR::Series0::Spi;
        using Reset  = AVR::Series0::Reset;
        using WatchDog = AVR::Series0::WatchDog;
        
        using PortRegister = AVR::Series1::PortRegister;
        using VPort = AVR::Series1::VPort;
        using Portmux = AVR::Series1::Portmux;
        using Ccl = AVR::Series2::Ccl;
        using Events = AVR::Series1::Events;
        using TCD = AVR::Series1::TCD;

        using SigRow = AVR::Series0::SigRow;
    };
    template<>
    constexpr bool ATTiny1624::is_atomic<uint8_t>() {return true;}
}

namespace AVR {
    template<> struct AVR::Component::Count<ATTiny1624::Gpior> : std::integral_constant<uint8_t, 4> {};
    template<> struct AVR::Component::Count<ATTiny1624::Usart> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATTiny1624::TCA> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATTiny1624::TCB> : std::integral_constant<uint8_t, 2> {};
    template<> struct AVR::Component::Count<ATTiny1624::TCD> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATTiny1624::Rtc> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATTiny1624::PortRegister> : std::integral_constant<uint8_t, 6> {};
    template<> struct AVR::Component::Count<ATTiny1624::Portmux> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATTiny1624::Ccl> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATTiny1624::Adc> : std::integral_constant<uint8_t, 2> {};
    template<> struct AVR::Component::Count<ATTiny1624::Spi> : std::integral_constant<uint8_t, 1> {};

    template<uint8_t N>
    struct ATTiny1624::Gpior::Address {
        inline static constexpr uintptr_t value = 0x001c + N;
    };
    
    template<> struct ATTiny1624::VPort::Address<A> {
        inline static constexpr uintptr_t value = 0x0000;
    };
    template<> struct ATTiny1624::VPort::Address<B> {
        inline static constexpr uintptr_t value = 0x0004;
    };
    template<> struct ATTiny1624::VPort::Address<C> {
        inline static constexpr uintptr_t value = 0x0008;
    };
    template<> struct ATTiny1624::VPort::Address<D> {
        inline static constexpr uintptr_t value = 0x000c;
    };
    template<> struct ATTiny1624::VPort::Address<E> {
        inline static constexpr uintptr_t value = 0x0010;
    };
    template<> struct ATTiny1624::VPort::Address<F> {
        inline static constexpr uintptr_t value = 0x0014;
    };
    
    template<> struct ATTiny1624::Spi::Address<0> {
        inline static constexpr uintptr_t value = 0x0820;
    };
    template<> struct ATTiny1624::Adc::Address<0> {
        inline static constexpr uintptr_t value = 0x0600;
    };
    template<> struct ATTiny1624::Adc::Address<1> {
        inline static constexpr uintptr_t value = 0x0640;
    };
    template<> struct ATTiny1624::Usart::Address<0> {
        inline static constexpr uintptr_t value = 0x0800;
    };
    template<> struct ATTiny1624::TCA::Address<0> {
        inline static constexpr uintptr_t value = 0x0A00;
    };
    template<> struct ATTiny1624::TCB::Address<0> {
        inline static constexpr uintptr_t value = 0x0A40;
    };
    template<> struct ATTiny1624::TCB::Address<1> {
        inline static constexpr uintptr_t value = 0x0A50;
    };
    template<> struct ATTiny1624::TCD::Address<0> {
        inline static constexpr uintptr_t value = 0x0A80;
    };
    template<> struct ATTiny1624::PortRegister::Address<A> {
        inline static constexpr uintptr_t value = 0x0400;
    };
    template<> struct ATTiny1624::PortRegister::Address<B> {
        inline static constexpr uintptr_t value = 0x0420;
    };

}
#pragma pack(pop)
#pragma once
