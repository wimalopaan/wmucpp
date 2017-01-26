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

struct ATMega328P final
{
    ATMega328P() = delete;
    struct Usart {
        static constexpr const uint8_t count = 1;
        enum class UCSRA : uint8_t {
            rxc = (1 << RXC0),
            txc = (1 << TXC0),
            udre = (1 << UDRE0),
            fe = (1 << FE0),
            dor = (1 << DOR0),
            upe = (1 << UPE0),
            u2x = (1 << U2X0),
            mpcm = (1 << MPCM0)
        };
        ControlRegister<Usart, UCSRA> ucsra;
        volatile uint8_t ucsrb;
        volatile uint8_t ucsrc;
        volatile uint8_t reserved1;
        union {
            struct {
                volatile uint8_t ubbrl;
                volatile uint8_t ubbrh;
            };
            volatile uint16_t ubbr;
        };
        DataRegister<Usart, ReadWrite> udr;
        volatile uint8_t reserved2;
        template<int N> struct Address;
    };
    struct Timer8Bit {
        static constexpr const uint8_t count = 2;
        typedef uint8_t value_type;
        enum class TCCRA : uint8_t {
            coma0 = (1 << COM0A0),
            coma1 = (1 << COM0A1),
            comb0 = (1 << COM0B0),
            comb1 = (1 << COM0B1),
            wgm0 = (1 << WGM00),
            wgm1 = (1 << WGM01)
        };
        ControlRegister<Timer8Bit, TCCRA> tccra;
        enum class TCCRB : uint8_t {
            foca = (1 << FOC0A),
            focb = (1 << FOC0B),
            wgm2 = (1 << WGM02),
            cs2 = (1 << CS02),
            cs1 = (1 << CS01),
            cs0 = (1 << CS00),
        };
        
        ControlRegister<Timer8Bit, TCCRB> tccrb;
        DataRegister<Timer8Bit, ReadWrite> tcnt;
        DataRegister<Timer8Bit, ReadWrite> ocra;
        DataRegister<Timer8Bit, ReadWrite> ocrb;
        template<int N> struct Address;
        template<int N> struct PrescalerBits;
    };

    struct Timer16Bit {
        static constexpr const uint8_t count = 1;
        typedef uint16_t value_type;
        enum class TCCRA : uint8_t {
            coma0 = (1 << COM1A0),
            coma1 = (1 << COM1A1),
            comb0 = (1 << COM1B0),
            comb1 = (1 << COM1B1),
            wgm0 = (1 << WGM10),
            wgm1 = (1 << WGM11)
        };
        ControlRegister<Timer16Bit, TCCRA> tccra;
        enum class TCCRB : uint8_t {
            icnc = (1 << ICNC1),
            ices = (1 << ICES1),
            wgm3 = (1 << WGM13),
            wgm2 = (1 << WGM12),
            cs2 = (1 << CS12),
            cs1 = (1 << CS11),
            cs0 = (1 << CS10),
        };
        ControlRegister<Timer16Bit, TCCRB> tccrb;
        volatile uint8_t tccrc;
        volatile uint8_t reserved;
        DataRegister<Timer16Bit, ReadWrite, uint16_t> tcnt;
        union {
            struct {
                volatile uint8_t icrl;
                volatile uint8_t icrh;
            };
            volatile uint16_t icr;
        };
        DataRegister<Timer16Bit, ReadWrite, uint16_t> ocra;
        DataRegister<Timer16Bit, ReadWrite, uint16_t> ocrb;
        template<int N> struct Address;
        template<int N> struct PrescalerBits;
    };
    struct PCInterrupts {
        volatile uint8_t pcmsk;
        template<int N> struct Address;
    };
    struct TimerInterrupts {
        volatile uint8_t tifr;
        volatile uint8_t padding[0x6E - 0x35 - 1];
        volatile uint8_t timsk;
        template<uint8_t N> struct Address {
            static constexpr uint8_t value = 0x35 + N;
        };
        template<uint8_t N> struct Flags;       
    };
    struct Spi {
        static constexpr const uint8_t count = 1;
        volatile uint8_t spcr;
        volatile uint8_t spsr;
        volatile uint8_t spdr;
        template<int N> struct Address;
    };
    struct PortRegister {
        volatile uint8_t in;
        volatile uint8_t ddr;
        volatile uint8_t out;
        template<typename P> struct Address;
    };
    struct Interrupt {
        volatile uint8_t pcifr;
        volatile uint8_t eifr;
        volatile uint8_t eimsk;
        volatile uint8_t padding[0x68 - 0x3b - 1 - 2];
        volatile uint8_t pcicr;
        volatile uint8_t eicra;
        static constexpr uint8_t address = 0x3b;
    };
};
}

namespace std {
template<>
struct enable_bitmask_operators<AVR::ATMega328P::Usart::UCSRA> {
    static constexpr bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATMega328P::Timer8Bit::TCCRA> {
    static constexpr bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATMega328P::Timer8Bit::TCCRB> {
    static constexpr bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATMega328P::Timer16Bit::TCCRA> {
    static constexpr bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATMega328P::Timer16Bit::TCCRB> {
    static constexpr bool enable = true;
};
}

namespace AVR {
    
template<>
struct ATMega328P::TimerInterrupts::Flags<0> {
    static constexpr uint8_t ociea = _BV(OCIE0A);
    static constexpr uint8_t toie = _BV(TOIE0);
};
template<>
struct ATMega328P::TimerInterrupts::Flags<1> {
    static constexpr uint8_t ociea = _BV(OCIE1A);
};
template<>
struct ATMega328P::TimerInterrupts::Flags<2> {
    static constexpr uint8_t ociea = _BV(OCIE2A);
    static constexpr uint8_t toie = _BV(TOIE2);
};

template<>
struct ATMega328P::PortRegister::Address<B> {
    static constexpr uint8_t value = 0x23;
};
template<>
struct ATMega328P::PortRegister::Address<C> {
    static constexpr uint8_t value = 0x26;
};
template<>
struct ATMega328P::PortRegister::Address<D> {
    static constexpr uint8_t value = 0x29;
};

template<>
struct ATMega328P::PCInterrupts::Address<0> {
    static constexpr uint8_t value = 0x6b;
};
template<>
struct ATMega328P::PCInterrupts::Address<1> {
    static constexpr uint8_t value = 0x6c;
};
template<>
struct ATMega328P::PCInterrupts::Address<2> {
    static constexpr uint8_t value = 0x6d;
};

template<>
struct ATMega328P::Spi::Address<0> {
    static constexpr uint8_t value = 0x4c;
};
template<>
struct ATMega328P::Usart::Address<0> {
    static constexpr uint8_t value = 0xc0;
};

//Timer0
template<>
struct ATMega328P::Timer8Bit::Address<0> {
    static constexpr uint8_t value = 0x44;
};

template<>
struct ATMega328P::Timer8Bit::PrescalerBits<0> {
    static constexpr auto values = AVR::prescalerValues10Bit<ATMega328P::Timer8Bit::TCCRB>;
};

// Timer2
template<>
struct ATMega328P::Timer8Bit::Address<2> {
    static constexpr uint8_t value = 0xb0;
};
template<>
struct ATMega328P::Timer8Bit::PrescalerBits<2> {
    static constexpr auto values = AVR::prescalerValues10BitExtended<ATMega328P::Timer8Bit::TCCRB>;
};

// timer 1
template<>
struct ATMega328P::Timer16Bit::Address<1> {
    static constexpr uint8_t value = 0x80;
};

template<>
struct ATMega328P::Timer16Bit::PrescalerBits<1> {
    static constexpr auto values = AVR::prescalerValues10Bit<ATMega328P::Timer16Bit::TCCRB>;
};

}
#pragma pack(pop)
