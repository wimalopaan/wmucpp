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

#include <stdint.h>

template<typename Component, int Number>
inline Component* getBaseAddr() {
    return reinterpret_cast<Component*>(Component::template Address<Number>::value);
}

struct MCU {
    struct Gpio {
        volatile uint8_t ddr;
        volatile uint8_t in;
        volatile uint8_t out;
        template<int N> struct Address;
    };
};
template<>
struct MCU::Gpio::Address<0> {
    static constexpr uintptr_t value = 0x55;
};

template<typename Port, uint8_t PinNumber>
struct Pin {
    static constexpr auto gpio = getBaseAddr<MCU::Gpio, 0>;
    static void init() {
        gpio()->ddr |= (1 << PinNumber);
        gpio()->out |= (1 << PinNumber);
    }
};

int main() {
    using led = Pin<MCU::Gpio, 0>;
    led::init();
    while(true) {}
}
