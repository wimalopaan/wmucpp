/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <stdint.h>

#include "avr8defs.h"

#pragma pack(push)
#pragma pack(1)

namespace AVR {

struct ATTiny84 final {
    ATTiny84() = delete;
    struct Timer8Bit {
        enum class TCCRA : uint8_t {
            coma0 = (1 << COM0A0),
            coma1 = (1 << COM0A1),
            comb0 = (1 << COM0B0),
            comb1 = (1 << COM0B1),
            wgm0 = (1 << WGM00),
            wgm1 = (1 << WGM01)
        };
        ControlRegister<Timer8Bit, TCCRA> tccra;
        volatile uint8_t unused1;
        DataRegister<Timer8Bit, ReadWrite> tcnt;
        enum class TCCRB : uint8_t {
            foca = (1 << FOC0A),
            focb = (1 << FOC0B),
            wgm2 = (1 << WGM02),
            cs2 = (1 << CS02),
            cs1 = (1 << CS01),
            cs0 = (1 << CS00),
        };
        ControlRegister<Timer8Bit, TCCRB> tccrb;
        volatile uint8_t unused2;
        volatile uint8_t unused3;
        DataRegister<Timer8Bit, ReadWrite> ocra;
        volatile uint8_t padding[0x3c - 0x36 - 1];
        DataRegister<Timer8Bit, ReadWrite> ocrb;
        template<int N> struct Address;
        template<int N> struct PrescalerBits;
    };
    
    struct USI {
        enum class USIC : uint8_t {
            sie = (1 << USISIE),
            oie = (1 << USIOIE),
            wm1 = (1 << USIWM1),
            wm0 = (1 << USIWM0),
            cs1 = (1 << USICS1),
            cs0 = (1 << USICS0),
            clk = (1 << USICLK),
            tc  = (1 << USITC)
        };
        ControlRegister<USI, USIC> usicr;
//        volatile uint8_t usicr;
        enum class USIS : uint8_t {
            sif  = (1 << USISIF),            
            oif  = (1 << USIOIF),            
            pf   = (1 << USIPF),            
            dc   = (1 << USIDC),            
            cnt3 = (1 << USICNT3),            
            cnt2 = (1 << USICNT2),            
            cnt1 = (1 << USICNT1),            
            cnt0 = (1 << USICNT0) 
        };
        ControlRegister<USI, USIS> usisr;
//        volatile uint8_t usisr;
        DataRegister<USI, ReadWrite> usidr;
        DataRegister<USI, ReadWrite> usibr;
        template<int N> struct Address;
    };

    struct PortRegister {
        volatile uint8_t in;
        volatile uint8_t ddr;
        volatile uint8_t out;
        template<typename P> struct Address;
    };
    struct TimerInterrupts {
        volatile uint8_t tifr;
        volatile uint8_t timsk;
        template<uint8_t N> struct Address;
        template<uint8_t N> struct Flags;       
    };
    struct Interrupt {
        volatile uint8_t gifr;
        volatile uint8_t gimsk;
        static constexpr uint8_t address = 0x5a;
    };
    struct PCInterrupts {
        volatile uint8_t pcmsk;
        template<int N> struct Address;
    };
};

template<>
struct ATTiny84::PCInterrupts::Address<0> {
    static constexpr uint8_t value = 0x32;
};
template<>
struct ATTiny84::PCInterrupts::Address<1> {
    static constexpr uint8_t value = 0x40;
};

template<>
struct ATTiny84::USI::Address<0> {
    static constexpr uint8_t value = 0x2d;
};

template<>
struct ATTiny84::PortRegister::Address<B> {
    static constexpr uint8_t value = 0x36;
};
template<>
struct ATTiny84::PortRegister::Address<A> {
    static constexpr uint8_t value = 0x39;
};

template<>
struct ATTiny84::Timer8Bit::Address<0> {
    static constexpr uint8_t value = 0x50;
};
}
namespace std {
template<>
struct enable_bitmask_operators<AVR::ATTiny84::USI::USIC> {
    static constexpr bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATTiny84::USI::USIS> {
    static constexpr bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATTiny84::Timer8Bit::TCCRA> {
    static constexpr bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATTiny84::Timer8Bit::TCCRB> {
    static constexpr bool enable = true;
};
}

namespace AVR {
template<>
struct ATTiny84::Timer8Bit::PrescalerBits<0> {
        static constexpr auto values = AVR::prescalerValues10Bit<ATTiny84::Timer8Bit::TCCRB>;
};

}
#pragma pack(pop)

