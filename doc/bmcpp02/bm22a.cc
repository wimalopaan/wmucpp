/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <stdint.h>
#include "std/byte.h"
#include "mcu/avr8.h"
#include "mcu/register.h"

template<typename Component, int Number>
inline Component* getBaseAddr() {
    return reinterpret_cast<Component*>(Component::template Address<Number>::value);
}

struct MCUx {
    struct Gpio {
        AVR::DataRegister<Gpio, AVR::ReadWrite, std::byte> ddr;
        AVR::DataRegister<Gpio, AVR::ReadWrite, std::byte> in;
        AVR::DataRegister<Gpio, AVR::ReadWrite, std::byte> out;
        template<int N> struct Address;
    };
};
template<>
struct MCUx::Gpio::Address<0> {
    static constexpr uintptr_t value = 0x30;
};

template<typename Port, uint8_t PinNumber>
struct Pin {
    static constexpr auto gpio = getBaseAddr<MCUx::Gpio, 0>;
    static void init() {
        *gpio()->ddr |= std::byte(1 << PinNumber);
        *gpio()->out |= std::byte(1 << PinNumber);
    }
};

int main() {
    using led = Pin<MCUx::Gpio, 0>;
    led::init();
    while(true) {}
}

