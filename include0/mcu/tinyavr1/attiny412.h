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

#include "components1.h"

#pragma pack(push)
#pragma pack(1)

namespace AVR {
    struct ATTiny412 final {
        
        template<typename T>
        inline static constexpr bool is_atomic() {return false;}
        
        ATTiny412() = delete;
        
        struct Ram final {
            inline static constexpr uintptr_t begin = RAMSTART;  
            inline static constexpr uintptr_t end   = RAMEND;  
        };
        
        using Cpu = AVR::Series0::Cpu;
        using Clock = AVR::Series0::Clock;
        using Rtc = AVR::Series0::Rtc;
        using Usart = AVR::Series0::Usart;
        using TCA = AVR::Series0::TCA;
        using TCB = AVR::Series0::TCB;
        using Sleep = AVR::Series0::Sleep;
        using Adc = AVR::Series0::Adc;
        using Vref = AVR::Series1::Vref;
        
        using PortRegister = AVR::Series1::PortRegister;
        using Portmux = AVR::Series1::Portmux;
        using Ccl = AVR::Series1::Ccl;
    };
    template<>
    constexpr bool ATTiny412::is_atomic<uint8_t>() {return true;}
}

namespace AVR {
    template<> struct AVR::Component::Count<ATTiny412::Usart> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATTiny412::TCA> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATTiny412::TCB> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATTiny412::Rtc> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATTiny412::PortRegister> : std::integral_constant<uint8_t, 6> {};
    template<> struct AVR::Component::Count<ATTiny412::Portmux> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATTiny412::Ccl> : std::integral_constant<uint8_t, 1> {};
    template<> struct AVR::Component::Count<ATTiny412::Adc> : std::integral_constant<uint8_t, 1> {};

    template<> struct ATTiny412::Adc::Address<0> {
        inline static constexpr uintptr_t value = 0x0600;
    };
    template<> struct ATTiny412::Usart::Address<0> {
        inline static constexpr uintptr_t value = 0x0800;
    };
    template<> struct ATTiny412::TCA::Address<0> {
        inline static constexpr uintptr_t value = 0x0A00;
    };
    template<> struct ATTiny412::TCB::Address<0> {
        inline static constexpr uintptr_t value = 0x0A40;
    };
    template<> struct ATTiny412::PortRegister::Address<A> {
        inline static constexpr uintptr_t value = 0x0400;
    };

}
#pragma pack(pop)
