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

// todo: make addresses uintptr_t

struct ATMega1284P final 
{
    ATMega1284P() = delete;
    
    template<typename T>
    static constexpr bool is_atomic() {return false;}
    
    struct Usart {
        static constexpr const uint8_t count = 2;
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
    struct TWI {
        static constexpr const uint8_t count = 1;
        volatile uint8_t twbr;
        volatile uint8_t twsr;
        volatile uint8_t twar;
        volatile uint8_t twdr;
        volatile uint8_t twcr;
        volatile uint8_t twamr;
        template<int N> struct Address;
        template<int N> struct PrescalerRow;
        template<int N, int F> struct Prescaler;
    };
    // todo: finish all ControlRegister and DataRegister
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
//        volatile uint8_t tccra;

        enum class TCCRB : uint8_t {
            foca = (1 << FOC0A),
            focb = (1 << FOC0B),
            wgm2 = (1 << WGM02),
            cs2 = (1 << CS02),
            cs1 = (1 << CS01),
            cs0 = (1 << CS00),
        };
        ControlRegister<Timer8Bit, TCCRB> tccrb;
//        volatile uint8_t tccrb;
        volatile uint8_t tcnt;
        volatile uint8_t ocra;
        volatile uint8_t ocrb;
        template<int N> struct Address;
        template<int N> struct PrescalerBits;
    };

    struct Timer16Bit {
        static constexpr const uint8_t count = 2;
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

//        volatile uint8_t tccra;
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
//        volatile uint8_t tccrb;
        volatile uint8_t tccrc;
        volatile uint8_t reserved;
        union {
            struct {
                volatile uint8_t tcntl;
                volatile uint8_t tcnth;
            };
            volatile uint16_t tcnt;
        };
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

    struct Adc {
        static constexpr const uint8_t count = 1;
        union {
            struct {
                volatile uint8_t adcl;
                volatile uint8_t adch;
            };
            volatile uint16_t adc;
        };
        volatile uint8_t adcsra;
        volatile uint8_t adcsrb;
        volatile uint8_t admux;
        volatile uint8_t reserved;
        volatile uint8_t didr0;
        volatile uint8_t didr1;
        template<int N> struct Address;
        template<int N> struct Parameter;
    };

    struct AdComparator {
        static constexpr const uint8_t count = 1;
        volatile uint8_t acsr;
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
constexpr bool ATMega1284P::is_atomic<uint8_t>() {return true;}

}

namespace std {
template<>
struct enable_bitmask_operators<AVR::ATMega1284P::Timer8Bit::TCCRA> {
    static constexpr bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATMega1284P::Timer8Bit::TCCRB> {
    static constexpr bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATMega1284P::Timer16Bit::TCCRA> {
    static constexpr bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATMega1284P::Timer16Bit::TCCRB> {
    static constexpr bool enable = true;
};
}

namespace AVR {

template<>
struct ATMega1284P::TimerInterrupts::Flags<0> {
    static constexpr uint8_t ociea = _BV(OCIE0A);
    static constexpr uint8_t toie = _BV(TOIE0);
};
template<>
struct ATMega1284P::TimerInterrupts::Flags<1> {
    static constexpr uint8_t ociea = _BV(OCIE1A);
};
template<>
struct ATMega1284P::TimerInterrupts::Flags<2> {
    static constexpr uint8_t ociea = _BV(OCIE2A);
    static constexpr uint8_t toie = _BV(TOIE2);
};

template<>
struct ATMega1284P::Adc::Parameter<0> {
    static constexpr uint8_t numberOfChannels = 8;
};

template<>
struct ATMega1284P::PortRegister::Address<A> {
    static constexpr uint8_t value = 0x20;
};
template<>
struct ATMega1284P::PortRegister::Address<B> {
    static constexpr uint8_t value = 0x23;
};
template<>
struct ATMega1284P::PortRegister::Address<C> {
    static constexpr uint8_t value = 0x26;
};
template<>
struct ATMega1284P::PortRegister::Address<D> {
    static constexpr uint8_t value = 0x29;
};

template<>
struct ATMega1284P::PCInterrupts::Address<0> {
    static constexpr uint8_t value = 0x6b;
};
template<>
struct ATMega1284P::PCInterrupts::Address<1> {
    static constexpr uint8_t value = 0x6c;
};
template<>
struct ATMega1284P::PCInterrupts::Address<2> {
    static constexpr uint8_t value = 0x6d;
};
template<>
struct ATMega1284P::PCInterrupts::Address<3> {
    static constexpr uint8_t value = 0x73;
};

template<>
struct ATMega1284P::Adc::Address<0> {
    static constexpr uint8_t value = 0x78;
};

template<>
struct ATMega1284P::AdComparator::Address<0> {
    static constexpr uint8_t value = 0x50;
};

template<>
struct ATMega1284P::TWI::Address<0> {
    static constexpr uint8_t value = 0xb8;
};

// todo: zusammenfassen wie bei Timer
template<>
struct ATMega1284P::TWI::PrescalerRow<0> {
    static constexpr uint8_t values[] = {1, 4, 16, 64};
};
template<>
struct ATMega1284P::TWI::Prescaler<0, 1> {
    static constexpr uint8_t value = 0x00;
};
template<>
struct ATMega1284P::TWI::Prescaler<0, 4> {
    static constexpr uint8_t value = 0x01;
};
template<>
struct ATMega1284P::TWI::Prescaler<0, 16> {
    static constexpr uint8_t value = 0x02;
};
template<>
struct ATMega1284P::TWI::Prescaler<0, 64> {
    static constexpr uint8_t value = 0x03;
};

template<>
struct ATMega1284P::Spi::Address<0> {
    static constexpr uint8_t value = 0x4c;
};
template<>
struct ATMega1284P::Usart::Address<0> {
    static constexpr uint8_t value = 0xc0;
};
template<>
struct ATMega1284P::Usart::Address<1> {
    static constexpr uint8_t value = 0xc8;
};

//Timer0
template<>
struct ATMega1284P::Timer8Bit::Address<0> {
    static constexpr uint8_t value = 0x44;
};
template<>
struct ATMega1284P::Timer8Bit::PrescalerBits<0> {
    static constexpr auto values = prescalerValues10Bit<ATMega1284P::Timer8Bit::TCCRB>;
};

// Timer2
template<>
struct ATMega1284P::Timer8Bit::Address<2> {
    static constexpr uint8_t value = 0xb0;
};
template<>
struct ATMega1284P::Timer8Bit::PrescalerBits<2> {
    static constexpr auto values = prescalerValues10BitExtended<ATMega1284P::Timer8Bit::TCCRB>;
};

// Timer 1 (16bit)
template<>
struct ATMega1284P::Timer16Bit::Address<1> {
    static constexpr uint8_t value = 0x80;
};
template<>
struct ATMega1284P::Timer16Bit::PrescalerBits<1> {
    static constexpr auto values = prescalerValues10Bit<ATMega1284P::Timer16Bit::TCCRB>;
};

// Timer 3 (16bit)
template<>
struct ATMega1284P::Timer16Bit::Address<3> {
    static constexpr uint8_t value = 0x90;
};
template<>
struct ATMega1284P::Timer16Bit::PrescalerBits<3> {
    static constexpr auto values = prescalerValues10Bit<ATMega1284P::Timer16Bit::TCCRB>;
};

}

#pragma pack(pop)
