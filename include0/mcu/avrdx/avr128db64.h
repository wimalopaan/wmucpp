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

#include "../common/components_db64.h"

#pragma pack(push)
#pragma pack(1)

namespace AVR {
    struct Avr128db64 final {
        template<typename T> inline static constexpr bool is_atomic() {return false;}
        
        Avr128db64() = delete;
        
        struct Ram final {
            inline static constexpr uintptr_t begin = RAMSTART;  
            inline static constexpr uintptr_t end   = RAMEND;  
        };
        
        using Cpu = AVR::Series0::Cpu;
        using Clock = AVR::SeriesDb::Clock;
        using Rtc = AVR::SeriesDb::Rtc;
        using Usart = AVR::SeriesDb::Usart;
        using PortRegister = AVR::SeriesDb::PortRegister;
        using VPort = AVR::SeriesDb::VPort;
        using Portmux= AVR::SeriesDb::Portmux;
        using Vref = AVR::SeriesDb::Vref;
        using Spi = AVR::Series0::Spi;
        using Dac = AVR::SeriesDb::Dac;
        
        using Twi = AVR::SeriesDa::Twi;
        using TCA = AVR::Series0::TCA;
        using TCB = AVR::SeriesDa::TCB;
        using TCD = AVR::Series1::TCD;
        using Ccl = AVR::SeriesDa::Ccl;
        using Events = AVR::SeriesDa::Events;
        using Adc = AVR::SeriesDa::Adc;
        using AdComparator = AVR::Series0::AdComparator;
        using Sleep = AVR::Series0::Sleep;
        using SigRow = AVR::SeriesDa::SigRow;
        using Gpior = AVR::Series0::GPIOR;
        using SysCfg = AVR::SeriesDa::SysCfg;
        using NvmCtrl = AVR::SeriesDa::NvmCtrl;
        
    };
    template<>
    constexpr bool Avr128db64::is_atomic<uint8_t>() {return true;}
}

namespace AVR {
    template<> struct AVR::Component::Count<Avr128db64::Gpior> : std::integral_constant<uint8_t, 4> {};
    template<> struct AVR::Component::Count<Avr128db64::Usart> : std::integral_constant<uint8_t, 6> {};
    template<> struct AVR::Component::Count<Avr128db64::TCA> : std::integral_constant<uint8_t, 2> {};
    template<> struct AVR::Component::Count<Avr128db64::TCB> : std::integral_constant<uint8_t, 5> {};
    template<> struct AVR::Component::Count<Avr128db64::TCD> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<Avr128db64::Rtc> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<Avr128db64::PortRegister> : std::integral_constant<uint8_t, 6> {};
    template<> struct AVR::Component::Count<Avr128db64::VPort> : std::integral_constant<uint8_t, 6> {};
    template<> struct AVR::Component::Count<Avr128db64::Portmux> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<Avr128db64::Ccl> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<Avr128db64::AdComparator> : std::integral_constant<uint8_t, 3> {};
    template<> struct AVR::Component::Count<Avr128db64::Adc> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<Avr128db64::Spi> : std::integral_constant<uint8_t, 2> {};
    template<> struct AVR::Component::Count<Avr128db64::NvmCtrl> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<Avr128db64::Twi> : std::integral_constant<uint8_t, 2> {};
    template<> struct AVR::Component::Count<Avr128db64::Dac> : std::integral_constant<uint8_t, 1> {};

    template<uint8_t N>
    struct Avr128db64::Gpior::Address {
        inline static constexpr uintptr_t value = 0x001c + N;
    };
    
    template<> struct Avr128db64::VPort::Address<A> {
        inline static constexpr uintptr_t value = 0x0000;
    };
    template<> struct Avr128db64::VPort::Address<B> {
        inline static constexpr uintptr_t value = 0x0004;
    };
    template<> struct Avr128db64::VPort::Address<C> {
        inline static constexpr uintptr_t value = 0x0008;
    };
    template<> struct Avr128db64::VPort::Address<D> {
        inline static constexpr uintptr_t value = 0x000c;
    };
    template<> struct Avr128db64::VPort::Address<E> {
        inline static constexpr uintptr_t value = 0x0010;
    };
    template<> struct Avr128db64::VPort::Address<F> {
        inline static constexpr uintptr_t value = 0x0014;
    };
    template<> struct Avr128db64::VPort::Address<G> {
        inline static constexpr uintptr_t value = 0x0018;
    };
    
    template<> struct Avr128db64::Spi::Address<0> {
        inline static constexpr uintptr_t value = 0x0940;
    };
    template<> struct Avr128db64::Spi::Address<1> {
        inline static constexpr uintptr_t value = 0x0960;
    };
    template<> struct Avr128db64::Adc::Address<0> {
        inline static constexpr uintptr_t value = 0x0600;
    };
    template<> struct Avr128db64::AdComparator::Address<0> {
        inline static constexpr uintptr_t value = 0x0680;
    };
    template<> struct Avr128db64::AdComparator::Address<1> {
        inline static constexpr uintptr_t value = 0x0688;
    };
    template<> struct Avr128db64::AdComparator::Address<2> {
        inline static constexpr uintptr_t value = 0x0690;
    };
    template<> struct Avr128db64::Dac::Address<0> {
        inline static constexpr uintptr_t value = 0x06a0;
    };
    template<> struct Avr128db64::Usart::Address<0> {
        inline static constexpr uintptr_t value = 0x0800;
    };
    template<> struct Avr128db64::Usart::Address<1> {
        inline static constexpr uintptr_t value = 0x0820;
    };
    template<> struct Avr128db64::Usart::Address<2> {
        inline static constexpr uintptr_t value = 0x0840;
    };
    template<> struct Avr128db64::Usart::Address<3> {
        inline static constexpr uintptr_t value = 0x0860;
    };
    template<> struct Avr128db64::Usart::Address<4> {
        inline static constexpr uintptr_t value = 0x0880;
    };
    template<> struct Avr128db64::Usart::Address<5> {
        inline static constexpr uintptr_t value = 0x08a0;
    };
    template<> struct Avr128db64::Twi::Address<0> {
        inline static constexpr uintptr_t value = 0x0900;
    };
    template<> struct Avr128db64::Twi::Address<1> {
        inline static constexpr uintptr_t value = 0x0920;
    };
    template<> struct Avr128db64::TCA::Address<0> {
        inline static constexpr uintptr_t value = 0x0A00;
    };
    template<> struct Avr128db64::TCA::Address<1> {
        inline static constexpr uintptr_t value = 0x0A40;
    };
    template<> struct Avr128db64::TCB::Address<0> {
        inline static constexpr uintptr_t value = 0x0B00;
    };
    template<> struct Avr128db64::TCB::Address<1> {
        inline static constexpr uintptr_t value = 0x0B10;
    };
    template<> struct Avr128db64::TCB::Address<2> {
        inline static constexpr uintptr_t value = 0x0B20;
    };
    template<> struct Avr128db64::TCB::Address<3> {
        inline static constexpr uintptr_t value = 0x0B30;
    };
    template<> struct Avr128db64::TCB::Address<4> {
        inline static constexpr uintptr_t value = 0x0B40;
    };
    template<> struct Avr128db64::PortRegister::Address<A> {
        inline static constexpr uintptr_t value = 0x0400;
    };
    template<> struct Avr128db64::PortRegister::Address<B> {
        inline static constexpr uintptr_t value = 0x0420;
    };
    template<> struct Avr128db64::PortRegister::Address<C> {
        inline static constexpr uintptr_t value = 0x0440;
    };
    template<> struct Avr128db64::PortRegister::Address<D> {
        inline static constexpr uintptr_t value = 0x0460;
    };
    template<> struct Avr128db64::PortRegister::Address<E> {
        inline static constexpr uintptr_t value = 0x0480;
    };
    template<> struct Avr128db64::PortRegister::Address<F> {
        inline static constexpr uintptr_t value = 0x04A0;
    };
    template<> struct Avr128db64::PortRegister::Address<G> {
        inline static constexpr uintptr_t value = 0x04C0;
    };
}
#pragma pack(pop)
