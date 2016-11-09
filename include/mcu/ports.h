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

#if __has_include(<avr/io.h>)
# include <avr/io.h>
#endif

#include "mcu/mcu.h"

// todo: Vordefinierte Pins aus pin-defs.txt generieren
// todo: mit __has_include inkludieren

namespace AVR {

struct Output final {
    Output() = delete;
    static constexpr bool value = true;
};
struct Input final {
    Input() = delete;
    static constexpr bool value = false;
};

template<typename MCUPort, typename Name>
struct Port final {
    typedef MCUPort mcuport_type;
    typedef Name name_type;
    Port() = delete;
    static void set(uint8_t v) {
        getBaseAddr<MCUPort, Name>()->out = v;
    }
    static volatile uint8_t& get() {
        return getBaseAddr<MCUPort, Name>()->out;
    }
    static void dir(uint8_t v) {
        getBaseAddr<MCUPort, Name>()->ddr = v;
    }
    static volatile uint8_t& dir() {
        return getBaseAddr<MCUPort, Name>()->ddr;
    }
    static volatile uint8_t& read() {
        return getBaseAddr<MCUPort, Name>()->in;
    }
    static constexpr uint16_t address() {
        return reinterpret_cast<uint16_t>(&getBaseAddr<MCUPort, Name>()->out);
    }
};

template<typename Port, uint8_t PinNumber>
struct Pin final {
    static_assert(PinNumber < 8, "wrong pin number");
    Pin() = delete;
    typedef Port port;
    static constexpr uint8_t number = PinNumber;
    static constexpr uint8_t pinMask = (1 << PinNumber);
    static void on() {
        Port::get() |= pinMask;
    }
    static constexpr auto& high = on;
    static constexpr auto& pullup = on;
    static void off() {
        Port::get() &= ~pinMask;
    }
    static constexpr auto& low = off;
    static void toggle() {
        Port::get() ^= pinMask;
    }
    template<typename Dir>
    static void dir() {
        if (Dir::value) {
            Port::dir() |= pinMask;
        }
        else {
            Port::dir() &= ~pinMask;
        }
    }
    static bool read() {
        return Port::read() & pinMask;
    }
    static constexpr auto& isHigh = read;
};

struct NoPin final {
    NoPin() = delete;
    static constexpr uint8_t number = 0;
    static constexpr uint8_t pinMask = 0x01;
    static void on() {}
    static constexpr auto& high = on;
    static void off() {}
    static constexpr auto& low = off;
    static void toggle() {}
    template<typename Dir>
    static void dir() {}
    static bool read() {return false;}
};


template<bool Bool, typename P>
void set(P){
    if(Bool == true) {
        getBaseAddr<typename P::mcuport_type, typename P::name_type>()->out = 0xff;;
    }
    else {
        getBaseAddr<typename P::mcuport_type, typename P::name_type>()->out = 0x00;;
    }
}

}
