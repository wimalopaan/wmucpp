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
        volatile uint8_t ubbrl;
        volatile uint8_t ubbrh;
        volatile uint8_t udr;
        volatile uint8_t reserved2;
        template<int N> struct Address;
    };
    struct Timer8Bit {
        static constexpr const uint8_t count = 2;
        volatile uint8_t tccra;
        volatile uint8_t tccrb;
        volatile uint8_t tcnt;
        volatile uint8_t ocra;
        volatile uint8_t ocrb;
        template<int N> struct Address;
        template<int N, int F> struct Prescaler;
        template<int N> struct PrescalerRow;
        template<uint8_t N> struct Flags; 
    };

    enum class Timer8BitTccra: uint8_t {
        None    = 0,
        WGM0    = 1 << 0,
        WGM1    = 1 << 1,
        COMB0   = 1 << 4,
        COMB1   = 1 << 5,
        COMA0   = 1 << 6,
        COMA1   = 1 << 7
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
        template<int F> struct Prescaler;
        template<int N> struct PrescalerRow;
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

#if defined(__AVR_ATmega328P__)

template<>
struct ATMega328P::TimerInterrupts::Flags<0> {
    static constexpr uint8_t ociea = _BV(OCIE0A);
};
template<>
struct ATMega328P::TimerInterrupts::Flags<1> {
    static constexpr uint8_t ociea = _BV(OCIE1A);
};
template<>
struct ATMega328P::TimerInterrupts::Flags<2> {
    static constexpr uint8_t ociea = _BV(OCIE2A);
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
template<>
struct ATMega328P::Timer8Bit::Address<0> {
    static constexpr uint8_t value = 0x44;
};
template<>
struct ATMega328P::Timer8Bit::Address<2> {
    static constexpr uint8_t value = 0xb0;
};

//Timer0
template<>
struct ATMega328P::Timer8Bit::Flags<0> {
    static constexpr uint8_t wgm1 = _BV(WGM01);
};
template<>
struct ATMega328P::Timer8Bit::Flags<2> {
    static constexpr uint8_t wgm1 = _BV(WGM01);
};
template<>
struct ATMega328P::Timer8Bit::PrescalerRow<0> {
    static constexpr uint16_t values[] = {1024, 256, 64, 8, 1};
};
template<>
struct ATMega328P::Timer8Bit::Prescaler<0, 0> {
    static constexpr uint8_t value = 0x00;
};
template<>
struct ATMega328P::Timer8Bit::Prescaler<0, 1> {
    static constexpr uint8_t value = _BV(CS00);
};
template<>
struct ATMega328P::Timer8Bit::Prescaler<0, 8> {
    static constexpr uint8_t value = _BV(CS01);
};
template<>
struct ATMega328P::Timer8Bit::Prescaler<0, 64> {
    static constexpr uint8_t value = _BV(CS01) | _BV(CS00);
};
template<>
struct ATMega328P::Timer8Bit::Prescaler<0, 256> {
    static constexpr uint8_t value = _BV(CS02);
};
template<>
struct ATMega328P::Timer8Bit::Prescaler<0, 1024> {
    static constexpr uint8_t value = _BV(CS02) | _BV(CS00);
};

// Timer2
template<>
struct ATMega328P::Timer8Bit::PrescalerRow<2> {
    static constexpr uint16_t values[] = {1, 8, 32, 64, 128, 256, 1024};
};
template<>
struct ATMega328P::Timer8Bit::Prescaler<2, 0> {
    static constexpr uint8_t value = 0x00;
};
template<>
struct ATMega328P::Timer8Bit::Prescaler<2, 1> {
    static constexpr uint8_t value = _BV(CS00);
};
template<>
struct ATMega328P::Timer8Bit::Prescaler<2, 8> {
    static constexpr uint8_t value = _BV(CS01);
};
template<>
struct ATMega328P::Timer8Bit::Prescaler<2, 32> {
    static constexpr uint8_t value = _BV(CS01) | _BV(CS00);
};
template<>
struct ATMega328P::Timer8Bit::Prescaler<2, 64> {
    static constexpr uint8_t value = _BV(CS02);
};
template<>
struct ATMega328P::Timer8Bit::Prescaler<2, 128> {
    static constexpr uint8_t value = _BV(CS02) | _BV(CS00);
};
template<>
struct ATMega328P::Timer8Bit::Prescaler<2, 256> {
    static constexpr uint8_t value = _BV(CS02) | _BV(CS01);
};
template<>
struct ATMega328P::Timer8Bit::Prescaler<2, 1024> {
    static constexpr uint8_t value = _BV(CS02) | _BV(CS01) | _BV(CS00);
};

// timer 1
template<>
struct ATMega328P::Timer16Bit::Address<1> {
    static constexpr uint8_t value = 0x80;
};
template<>
struct ATMega328P::Timer16Bit::PrescalerRow<1> {
    static constexpr uint16_t values[] = {1, 8, 64, 256, 1024};
};
template<>
struct ATMega328P::Timer16Bit::Prescaler<1> {
    static constexpr uint8_t value = _BV(CS10);
};
template<>
struct ATMega328P::Timer16Bit::Prescaler<8> {
    static constexpr uint8_t value = _BV(CS11);
};
template<>
struct ATMega328P::Timer16Bit::Prescaler<64> {
    static constexpr uint8_t value = _BV(CS11) | _BV(CS10);
};
template<>
struct ATMega328P::Timer16Bit::Prescaler<256> {
    static constexpr uint8_t value = _BV(CS12);
};
template<>
struct ATMega328P::Timer16Bit::Prescaler<1024> {
    static constexpr uint8_t value = _BV(CS12) | _BV(CS10);
};

#endif

}
#pragma pack(pop)
