/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
        volatile uint8_t tccra;
        volatile uint8_t unused1;
        volatile uint8_t tcnt;
        volatile uint8_t tccrb;
        volatile uint8_t unused2;
        volatile uint8_t unused3;
        volatile uint8_t ocra;
        volatile uint8_t padding[0x3c - 0x36 - 1];
        volatile uint8_t ocrb;
        template<int N> struct Address;
        template<int N> struct PrescalerBits;
        template<uint8_t N> struct Flags; 
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

template<>
struct ATTiny84::Timer8Bit::PrescalerBits<0> {
    static constexpr AVR::PrescalerPair values[] = {
        {_BV(CS02) |             _BV(CS00), 1024},
        {_BV(CS02)                        , 256},
        {            _BV(CS01) | _BV(CS00), 64},
        {            _BV(CS01)            , 8},
        {                        _BV(CS00), 1},
        {0                                , 0}
    };
};

}
#pragma pack(pop)

