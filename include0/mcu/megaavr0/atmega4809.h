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

#include "components0.h"

#pragma pack(push)
#pragma pack(1)

namespace AVR {
    struct ATMega4809 final {
        template<typename T> inline static constexpr bool is_atomic() {return false;}
        
        ATMega4809() = delete;
        
        struct Ram final {
            inline static constexpr uintptr_t begin = RAMSTART;  
            inline static constexpr uintptr_t end   = RAMEND;  
        };
        
        using Cpu = AVR::Series0::Cpu;
        using Clock = AVR::Series0::Clock;
        using Rtc = AVR::Series0::Rtc;
        using Usart = AVR::Series0::Usart;
        using PortRegister = AVR::Series0::PortRegister;
        using Portmux= AVR::Series0::Portmux;
        using TCA = AVR::Series0::TCA;
        using TCB = AVR::Series0::TCB;
        using Ccl = AVR::Series0::Ccl;
        using Events = AVR::Series0::Events;
        using Adc = AVR::Series0::Adc;
        using AdComparator = AVR::Series0::AdComparator;
        using Vref = AVR::Series0::Vref;
        using Sleep = AVR::Series0::Sleep;
        
    };
    template<>
    constexpr bool ATMega4809::is_atomic<uint8_t>() {return true;}
}

namespace AVR {
    template<> struct AVR::Component::Count<ATMega4809::Usart> : std::integral_constant<uint8_t, 4> {};
    template<> struct AVR::Component::Count<ATMega4809::TCA> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATMega4809::TCB> : std::integral_constant<uint8_t, 4> {};
    template<> struct AVR::Component::Count<ATMega4809::Rtc> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATMega4809::PortRegister> : std::integral_constant<uint8_t, 6> {};
    template<> struct AVR::Component::Count<ATMega4809::Portmux> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATMega4809::Ccl> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATMega4809::AdComparator> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATMega4809::Adc> : std::integral_constant<uint8_t, 1> {};
    
    template<> struct ATMega4809::Adc::Address<0> {
        inline static constexpr uintptr_t value = 0x0600;
    };
    template<> struct ATMega4809::AdComparator::Address<0> {
        inline static constexpr uintptr_t value = 0x0680;
    };
    template<> struct ATMega4809::Usart::Address<0> {
        inline static constexpr uintptr_t value = 0x0800;
    };
    template<> struct ATMega4809::Usart::Address<1> {
        inline static constexpr uintptr_t value = 0x0820;
    };
    template<> struct ATMega4809::Usart::Address<2> {
        inline static constexpr uintptr_t value = 0x0840;
    };
    template<> struct ATMega4809::Usart::Address<3> {
        inline static constexpr uintptr_t value = 0x0860;
    };
    template<> struct ATMega4809::TCA::Address<0> {
        inline static constexpr uintptr_t value = 0x0A00;
    };
    template<> struct ATMega4809::TCB::Address<0> {
        inline static constexpr uintptr_t value = 0x0A80;
    };
    template<> struct ATMega4809::TCB::Address<1> {
        inline static constexpr uintptr_t value = 0x0A90;
    };
    template<> struct ATMega4809::TCB::Address<2> {
        inline static constexpr uintptr_t value = 0x0Aa0;
    };
    template<> struct ATMega4809::TCB::Address<3> {
        inline static constexpr uintptr_t value = 0x0Ab0;
    };
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
    template<> struct ATMega4809::PortRegister::Address<E> {
        inline static constexpr uintptr_t value = 0x0480;
    };
    template<> struct ATMega4809::PortRegister::Address<F> {
        inline static constexpr uintptr_t value = 0x04A0;
    };
}
#pragma pack(pop)
