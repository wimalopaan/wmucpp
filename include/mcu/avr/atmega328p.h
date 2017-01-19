/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
        volatile uint8_t ucsra;
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
        volatile uint8_t udr;
        volatile uint8_t reserved2;
        template<int N> struct Address;
    };
    struct Timer8Bit {
        static constexpr const uint8_t count = 2;
        enum class TCCRA : uint8_t {
#ifdef COM0A0
            coma0 = (1 << COM0A0),
#endif
#ifdef COM0A1
            coma1 = (1 << COM0A1),
#endif
#ifdef COM0B0
            comb0 = (1 << COM0B0),
#endif
#ifdef COM0B1
            comb1 = (1 << COM0B1),
#endif
#ifdef WGM00
            wgm0 = (1 << WGM00),
#endif        
#ifdef WGM01
            wgm1 = (1 << WGM01)
#endif        
        };
        ControlRegister<Timer8Bit, TCCRA> tccra;
//        volatile uint8_t tccra;
        volatile uint8_t tccrb;
        volatile uint8_t tcnt;
        volatile uint8_t ocra;
        volatile uint8_t ocrb;
        template<int N> struct Address;
        template<int N> struct PrescalerBits;
        template<uint8_t N> struct Flags; 
    };

    struct Timer16Bit {
        static constexpr const uint8_t count = 1;
        volatile uint8_t tccra;
        volatile uint8_t tccrb;
        volatile uint8_t tccrc;
        volatile uint8_t reserved;
        volatile uint8_t tcntl;
        volatile uint8_t tcnth;
        union {
            struct {
                volatile uint8_t icrl;
                volatile uint8_t icrh;
            };
            volatile uint16_t icr;
        };
        union {
            struct {
                volatile uint8_t ocral;
                volatile uint8_t ocrah;
            };
            volatile uint16_t ocra;
        };
        union {
            struct {
                volatile uint8_t ocrbl;
                volatile uint8_t ocrbh;
            };
            volatile uint16_t ocrb;
        };
        template<int N> struct Address;
        template<int N> struct PrescalerBits;
        template<uint8_t N> struct Flags; 
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

template<>
struct ATMega328P::TimerInterrupts::Flags<0> {
#if defined(OCIE0A)
    static constexpr uint8_t ociea = _BV(OCIE0A);
#endif
#if defined(TOIE0)
    static constexpr uint8_t toie = _BV(TOIE0);
#endif
};
template<>
struct ATMega328P::TimerInterrupts::Flags<1> {
#if defined(OCIE1A)
    static constexpr uint8_t ociea = _BV(OCIE1A);
#endif
};
template<>
struct ATMega328P::TimerInterrupts::Flags<2> {
#if defined(OCIE2A)
    static constexpr uint8_t ociea = _BV(OCIE2A);
#endif
#if defined(TOIE2)
    static constexpr uint8_t toie = _BV(TOIE2);
#endif
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
struct ATMega328P::Timer8Bit::Flags<0> {
#if defined(WGM01)
    static constexpr uint8_t wgm1 = _BV(WGM01);
#endif
};
template<>
struct ATMega328P::Timer8Bit::PrescalerBits<0> {
    static constexpr AVR::PrescalerPair values[] = {
        {_BV(CS02) |             _BV(CS00), 1024},
        {_BV(CS02)                        , 256},
        {            _BV(CS01) | _BV(CS00), 64},
        {            _BV(CS01)            , 8},
        {                        _BV(CS00), 1},
        {0                                , 0}
    };
};

// Timer2
template<>
struct ATMega328P::Timer8Bit::Address<2> {
    static constexpr uint8_t value = 0xb0;
};
template<>
struct ATMega328P::Timer8Bit::Flags<2> {
#if defined(WGM21)
    static constexpr uint8_t wgm1 = _BV(WGM21);
#endif
};
template<>
struct ATMega328P::Timer8Bit::PrescalerBits<2> {
    static constexpr AVR::PrescalerPair values[] = {
        {_BV(CS02) | _BV(CS01) | _BV(CS00), 1024},
        {_BV(CS02) | _BV(CS01)            , 256},
        {_BV(CS02)             | _BV(CS00), 128},
        {_BV(CS02)                        , 64},
        {            _BV(CS01) | _BV(CS00), 32},
        {            _BV(CS01)            , 8},
        {                        _BV(CS00), 1},
        {0                                , 0}
    };
};

// timer 1
template<>
struct ATMega328P::Timer16Bit::Address<1> {
    static constexpr uint8_t value = 0x80;
};
template<>
struct ATMega328P::Timer16Bit::PrescalerBits<1> {
    static constexpr AVR::PrescalerPair values[] = {
        {_BV(CS12) |             _BV(CS10), 1024},
        {_BV(CS12)                        , 256},
        {            _BV(CS11) | _BV(CS10), 64},
        {            _BV(CS11)            , 8},
        {                        _BV(CS10), 1},
        {0                                , 0}
    };
};
template<>
struct ATMega328P::Timer16Bit::Flags<1> {
#if defined(WGM12)
    static constexpr uint8_t wgm2 = _BV(WGM12);
#endif
};

}
#pragma pack(pop)
