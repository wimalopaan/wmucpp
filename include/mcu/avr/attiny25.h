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

struct ATTiny25 final {
    ATTiny25() = delete;

    template<typename T>
    static constexpr bool is_atomic() {return false;}
    
    struct Timer8Bit {
        DataRegister<Timer8Bit> ocrb;
        DataRegister<Timer8Bit> ocra;
        enum class TCCRA : uint8_t {
            coma0 = (1 << COM0A0),
            coma1 = (1 << COM0A1),
            comb0 = (1 << COM0B0),
            comb1 = (1 << COM0B1),
            wgm0 = (1 << WGM00),
            wgm1 = (1 << WGM01)
        };
        
        ControlRegister<Timer8Bit, TCCRA> tccra;
        volatile uint8_t padding[0x32 - 0x2A - 1];
        volatile uint8_t tcnt;
        enum class TCCRB : uint8_t {
            foca = (1 << FOC0A),
            focb = (1 << FOC0B),
            wgm2 = (1 << WGM02),
            cs2 = (1 << CS02),
            cs1 = (1 << CS01),
            cs0 = (1 << CS00),
        };
        ControlRegister<Timer8Bit, TCCRB> tccrb;
        template<int N> struct Address;
        template<int N> struct PrescalerBits;
    };
    struct Timer8BitHighSpeed {
        volatile uint8_t ocrb;
        volatile uint8_t gtccr;
        volatile uint8_t ocrc;
        volatile uint8_t ocra;
        volatile uint8_t tcnt;
        enum class TCCR : uint8_t {
            ctc = (1 << CTC1),
            pwma = (1 << PWM1A),
            coma1 = (1 << COM1A1),
            coma0 = (1 << COM1A0),
            cs3 = (1 << CS13),
            cs2 = (1 << CS12),
            cs1 = (1 << CS11),
            cs0 = (1 << CS10),
        };
        ControlRegister<Timer8BitHighSpeed, TCCR> tccr;
        template<int N> struct Address;
        template<int N> struct PrescalerBits;
//        template<uint8_t N> struct Flags; 
    };
    
    struct USI {
        volatile uint8_t usicr;
        volatile uint8_t usisr;
        volatile uint8_t usidr;
        volatile uint8_t usibr;
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
constexpr bool ATTiny25::is_atomic<uint8_t>() {return true;}

template<>
struct ATTiny25::TimerInterrupts::Address<0> {
    static constexpr uint8_t value = 0x58;
};
template<>
struct ATTiny25::TimerInterrupts::Address<1> {
    static constexpr uint8_t value = 0x58;
};

template<>
struct ATTiny25::TimerInterrupts::Flags<0> {
    static constexpr uint8_t ociea = _BV(OCIE0A);
    static constexpr uint8_t toie = _BV(TOIE0);
};

template<>
struct ATTiny25::TimerInterrupts::Flags<1> {
    static constexpr uint8_t ociea = _BV(OCIE1A);
};


template<>
struct ATTiny25::PCInterrupts::Address<0> {
    static constexpr uint8_t value = 0x35;
};

template<>
struct ATTiny25::USI::Address<0> {
    static constexpr uint8_t value = 0x2d;
};

template<>
struct ATTiny25::PortRegister::Address<B> {
    static constexpr uint8_t value = 0x36;
};

template<>
struct ATTiny25::Timer8Bit::Address<0> {
    static constexpr uint8_t value = 0x48;
};
}
namespace std {
template<>
struct enable_bitmask_operators<AVR::ATTiny25::Timer8Bit::TCCRA> {
    static constexpr bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATTiny25::Timer8Bit::TCCRB> {
    static constexpr bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATTiny25::Timer8BitHighSpeed::TCCR> {
    static constexpr bool enable = true;
};
}

namespace AVR {

template<>
struct ATTiny25::Timer8Bit::PrescalerBits<0> {
    static constexpr auto values = prescalerValues10Bit<ATTiny25::Timer8Bit::TCCRB>;
};

template<>
struct ATTiny25::Timer8BitHighSpeed::Address<1> {
    static constexpr uint8_t value = 0x4B;
};
template<>
struct ATTiny25::Timer8BitHighSpeed::PrescalerBits<1> {
    static constexpr auto values = prescalerValues14Bit<ATTiny25::Timer8BitHighSpeed::TCCR>;
};

}
#pragma pack(pop)

