/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

#pragma pack(push)
#pragma pack(1)

namespace AVR {

// todo: prescaler rows?
struct ATMega8 final
{
    ATMega8() = delete;
    struct Usart {
        volatile uint8_t ubbrl;
        volatile uint8_t ucsrb;
        volatile uint8_t ucsra;
        volatile uint8_t udr;
        template<int N> struct Address;
    };
    struct Timer8Bit {
        volatile uint8_t tccr;
        volatile uint8_t tcnt;
        template<int N> struct Address;
        template<int F> struct Prescaler;
    };
    struct Timer16Bit {
        volatile uint8_t icrl;
        volatile uint8_t icrh;
        volatile uint8_t ocrbl;
        volatile uint8_t ocrbh;
        volatile uint8_t ocral;
        volatile uint8_t ocrah;
        volatile uint8_t tcntl;
        volatile uint8_t tcnth;
        volatile uint8_t tccrb;
        volatile uint8_t tccra;
        template<int N> struct Address;
        template<int F> struct Prescaler;
    };
    struct PortRegister {
        volatile uint8_t in;
        volatile uint8_t ddr;
        volatile uint8_t out;
        template<typename P> struct Address;
    };
    struct Interrupt {
        volatile uint8_t tifr;
        volatile uint8_t timsk;
        volatile uint8_t gifr;
        volatile uint8_t gicr;
        static constexpr uint8_t address = 0x58;
    };
    class TimerInterrupts {
    public:
        volatile uint8_t tifr;
        volatile uint8_t timsk;
        struct Address {
            static constexpr uint8_t value = 0x58;
        };
    };

};
template<>
struct ATMega8::PortRegister::Address<B> {
    static constexpr uint8_t value = 0x36;
};
template<>
struct ATMega8::PortRegister::Address<C> {
    static constexpr uint8_t value = 0x33;
};
template<>
struct ATMega8::PortRegister::Address<D> {
    static constexpr uint8_t value = 0x30;
};
template<>
struct ATMega8::Usart::Address<0> {
    static constexpr uint8_t value = 0x29;
};

template<>
struct ATMega8::Timer8Bit::Address<0> {
    static constexpr uint8_t value = 0x53;
};
template<>
struct ATMega8::Timer8Bit::Prescaler<0> {
    static constexpr uint8_t value = 0x00;
};
template<>
struct ATMega8::Timer8Bit::Prescaler<1> {
    static constexpr uint8_t value = _BV(CS00);
};
template<>
struct ATMega8::Timer8Bit::Prescaler<8> {
    static constexpr uint8_t value = _BV(CS01);
};
template<>
struct ATMega8::Timer8Bit::Prescaler<64> {
    static constexpr uint8_t value = _BV(CS01) | _BV(CS00);
};
template<>
struct ATMega8::Timer8Bit::Prescaler<256> {
    static constexpr uint8_t value = _BV(CS02);
};
template<>
struct ATMega8::Timer8Bit::Prescaler<1024> {
    static constexpr uint8_t value = _BV(CS02) | _BV(CS00);
};
template<>
struct ATMega8::Timer16Bit::Address<1> {
    static constexpr uint8_t value = 0x46;
};

}
#pragma pack(pop)
