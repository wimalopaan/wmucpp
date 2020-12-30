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
#include "../common/register.h"

#include "../common/components_da32.h"

#pragma pack(push)
#pragma pack(1)

namespace AVR {
    struct Avr128da32 final {
        template<typename T> inline static constexpr bool is_atomic() {return false;}
        
        Avr128da32() = delete;
        
        struct Ram final {
            inline static constexpr uintptr_t begin = RAMSTART;  
            inline static constexpr uintptr_t end   = RAMEND;  
        };
        
        using Cpu = AVR::Series0::Cpu;
        using Clock = AVR::SeriesDa::Clock;
        using Rtc = AVR::Series0::Rtc;
        using Usart = AVR::SeriesDa::Usart;
        using PortRegister = AVR::Series0::PortRegister;
        using VPort = AVR::Series0::VPort;
        using Portmux= AVR::SeriesDa::Portmux;
        using TCA = AVR::Series0::TCA;
        using TCB = AVR::SeriesDa::TCB;
        using TCD = AVR::Series1::TCD;
        using Ccl = AVR::SeriesDa::Ccl;
        using Events = AVR::SeriesDa::Events;
        using Adc = AVR::SeriesDa::Adc;
        using AdComparator = AVR::Series0::AdComparator;
        using Vref = AVR::SeriesDa::Vref;
        using Sleep = AVR::Series0::Sleep;
        using SigRow = AVR::SeriesDa::SigRow;
        using Spi = AVR::Series0::Spi;
        using Gpior = AVR::Series0::GPIOR;
        
    };
    template<>
    constexpr bool Avr128da32::is_atomic<uint8_t>() {return true;}
}

namespace AVR {
    template<> struct AVR::Component::Count<Avr128da32::Gpior> : std::integral_constant<uint8_t, 4> {};
    template<> struct AVR::Component::Count<Avr128da32::Usart> : std::integral_constant<uint8_t, 3> {};
    template<> struct AVR::Component::Count<Avr128da32::TCA> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<Avr128da32::TCB> : std::integral_constant<uint8_t, 3> {};
    template<> struct AVR::Component::Count<Avr128da32::TCD> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<Avr128da32::Rtc> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<Avr128da32::PortRegister> : std::integral_constant<uint8_t, 6> {};
    template<> struct AVR::Component::Count<Avr128da32::VPort> : std::integral_constant<uint8_t, 6> {};
    template<> struct AVR::Component::Count<Avr128da32::Portmux> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<Avr128da32::Ccl> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<Avr128da32::AdComparator> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<Avr128da32::Adc> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<Avr128da32::Spi> : std::integral_constant<uint8_t, 2> {};

    template<uint8_t N>
    struct Avr128da32::Gpior::Address {
        inline static constexpr uintptr_t value = 0x001c + N;
    };
    
    template<> struct Avr128da32::VPort::Address<A> {
        inline static constexpr uintptr_t value = 0x0000;
    };
    template<> struct Avr128da32::VPort::Address<B> {
        inline static constexpr uintptr_t value = 0x0004;
    };
    template<> struct Avr128da32::VPort::Address<C> {
        inline static constexpr uintptr_t value = 0x0008;
    };
    template<> struct Avr128da32::VPort::Address<D> {
        inline static constexpr uintptr_t value = 0x000c;
    };
    template<> struct Avr128da32::VPort::Address<E> {
        inline static constexpr uintptr_t value = 0x0010;
    };
    template<> struct Avr128da32::VPort::Address<F> {
        inline static constexpr uintptr_t value = 0x0014;
    };
//    template<> struct Avr128da32::VPort::Address<G> {
//        inline static constexpr uintptr_t value = 0x0018;
//    };

    
    template<> struct Avr128da32::Spi::Address<0> {
        inline static constexpr uintptr_t value = 0x0940;
    };
    template<> struct Avr128da32::Spi::Address<1> {
        inline static constexpr uintptr_t value = 0x0960;
    };
    template<> struct Avr128da32::Adc::Address<0> {
        inline static constexpr uintptr_t value = 0x0600;
    };
    template<> struct Avr128da32::AdComparator::Address<0> {
        inline static constexpr uintptr_t value = 0x0680;
    };
    template<> struct Avr128da32::AdComparator::Address<1> {
        inline static constexpr uintptr_t value = 0x0688;
    };
    template<> struct Avr128da32::AdComparator::Address<2> {
        inline static constexpr uintptr_t value = 0x0690;
    };
    template<> struct Avr128da32::Usart::Address<0> {
        inline static constexpr uintptr_t value = 0x0800;
    };
    template<> struct Avr128da32::Usart::Address<1> {
        inline static constexpr uintptr_t value = 0x0820;
    };
    template<> struct Avr128da32::Usart::Address<2> {
        inline static constexpr uintptr_t value = 0x0840;
    };
    template<> struct Avr128da32::Usart::Address<3> {
        inline static constexpr uintptr_t value = 0x0860;
    };
    template<> struct Avr128da32::TCA::Address<0> {
        inline static constexpr uintptr_t value = 0x0A00;
    };
    template<> struct Avr128da32::TCB::Address<0> {
        inline static constexpr uintptr_t value = 0x0B00;
    };
    template<> struct Avr128da32::TCB::Address<1> {
        inline static constexpr uintptr_t value = 0x0B10;
    };
    template<> struct Avr128da32::TCB::Address<2> {
        inline static constexpr uintptr_t value = 0x0B20;
    };
    template<> struct Avr128da32::TCB::Address<3> {
        inline static constexpr uintptr_t value = 0x0B30;
    };
    template<> struct Avr128da32::PortRegister::Address<A> {
        inline static constexpr uintptr_t value = 0x0400;
    };
    template<> struct Avr128da32::PortRegister::Address<B> {
        inline static constexpr uintptr_t value = 0x0420;
    };
    template<> struct Avr128da32::PortRegister::Address<C> {
        inline static constexpr uintptr_t value = 0x0440;
    };
    template<> struct Avr128da32::PortRegister::Address<D> {
        inline static constexpr uintptr_t value = 0x0460;
    };
    template<> struct Avr128da32::PortRegister::Address<E> {
        inline static constexpr uintptr_t value = 0x0480;
    };
    template<> struct Avr128da32::PortRegister::Address<F> {
        inline static constexpr uintptr_t value = 0x04A0;
    };
}
#pragma pack(pop)
