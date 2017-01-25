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

#pragma pack(push)
#pragma pack(1)

#include <stdint.h>

namespace AVR {

struct ATMegaNone final
{
    ATMegaNone() = delete;
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
        uint8_t in;
        uint8_t ddr;
        uint8_t& out = in;
        template<typename P> struct Address;
    };
    struct Interrupt {
        uint8_t tifr;
        uint8_t timsk;
        uint8_t gifr;
        uint8_t gicr;
        static constexpr uint8_t address = 0x0;
    };
    static PortRegister& portRegisterB() {
        static PortRegister reg;
        return reg;
    }
    static PortRegister& portRegisterC() {
        static PortRegister reg;
        return reg;
    }
    static PortRegister& portRegisterD() {
        static PortRegister reg;
        return reg;
    }
};

template<>
template<>
struct ATMegaNone::Timer8Bit::Prescaler<0> {
    static constexpr uint8_t value = 0x00;
};
template<>
struct ATMegaNone::Timer8Bit::Prescaler<1> {
    static constexpr uint8_t value = 0x00;
};
template<>
struct ATMegaNone::Timer8Bit::Prescaler<8> {
    static constexpr uint8_t value = 0x00;
};
template<>
struct ATMegaNone::Timer8Bit::Prescaler<64> {
    static constexpr uint8_t value = 0x00;
};
template<>
struct ATMegaNone::Timer8Bit::Prescaler<256> {
    static constexpr uint8_t value = 0x00;
};
template<>
struct ATMegaNone::Timer8Bit::Prescaler<1024> {
    static constexpr uint8_t value = 0x00;
};


template<>
AVR::ATMegaNone::PortRegister* getBaseAddr<AVR::ATMegaNone::PortRegister, B>() {
    return &AVR::ATMegaNone::portRegisterB();
}

template<>
AVR::ATMegaNone::PortRegister* getBaseAddr<AVR::ATMegaNone::PortRegister, C>() {
    return &AVR::ATMegaNone::portRegisterC();
}

template<>
AVR::ATMegaNone::PortRegister* getBaseAddr<AVR::ATMegaNone::PortRegister, D>() {
    return &AVR::ATMegaNone::portRegisterD();
}

}
#pragma pack(pop)
