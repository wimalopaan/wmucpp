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

#pragma once

#include <stdint.h>

#if __has_include(<avr/io.h>)
# include <avr/io.h>
#endif

#include "mcu/concepts.h"
#include "mcu/avr/avr8defs.h"

namespace AVR {

struct ATMegaNone;
struct ATMega8;
struct ATMega88P;
struct ATMega1284P;
struct ATMega328P;
struct ATMega328PB;
struct ATTiny84;
struct ATTiny85;
struct ATTiny25;

//template<typename T>
//concept bool ATMega_328PB() {
//    return std::is_same<T, ATMega328PB>::value;
//}

template<typename T>
concept bool ATTiny_X5() {
    return std::is_same<T, ATTiny25>::value || std::is_same<T, ATTiny85>::value;
}
template<typename T>
concept bool ATTiny_X4() {
    return std::is_same<T, ATTiny84>::value;
}
template<typename T>
concept bool ATMega_8() {
    return std::is_same<T, ATMega8>::value;
}
template<typename T>
concept bool ATMega_X8() {
    return std::is_same<T, ATMega328P>::value || std::is_same<T, ATMega328PB>::value || std::is_same<T, ATMega88P>::value;
}
template<typename T>
concept bool ATMega_X4() {
    return std::is_same<T, ATMega1284P>::value;
}

// todo: change to non constexpr here, since reinterpret_cast renders it non-constexpr at all

template<MCU::SingleComponent Component>
constexpr inline Component* getBaseAddr() {
    return reinterpret_cast<Component*>(Component::address);
}

template<MCU::MultipleComponent Component, int Number>
constexpr inline Component* getBaseAddr() {
    return reinterpret_cast<Component*>(Component::template Address<Number>::value);
}

template<typename Component, MCU::Letter Letter>
constexpr inline Component* getBaseAddr() {
    return reinterpret_cast<Component* const>(Component::template Address<Letter>::value);
}

}

#if defined(__AVR_ATmega1284P__)
# include "mcu/avr/atmega1284p.h"
typedef AVR::ATMega1284P DefaultMcuType;
#elif defined(__AVR_ATmega328P__)
# include "mcu/avr/atmega328p.h"
typedef AVR::ATMega328P DefaultMcuType;
#elif defined(__AVR_ATmega88P__)
# include "mcu/avr/atmega88p.h"
typedef AVR::ATMega88P DefaultMcuType;
#elif defined(__AVR_ATmega328PB__)
#include "mcu/avr/atmega328pb.h"
typedef AVR::ATMega328PB DefaultMcuType;
#elif defined(__AVR_ATmega8__)
#include "mcu/avr/atmega8.h"
typedef AVR::ATMega8 DefaultMcuType;
#elif defined(__AVR_ATtiny85__)
typedef AVR::ATTiny85 DefaultMcuType;
#include "mcu/avr/attiny85.h"
#elif defined(__AVR_ATtiny25__)
typedef AVR::ATTiny25 DefaultMcuType;
#include "mcu/avr/attiny25.h"
#elif defined(__AVR_ATtiny84__)
#include "mcu/avr/attiny84.h"
typedef AVR::ATTiny84 DefaultMcuType;
#else
typedef AVR::ATMegaNone DefaultMcuType;
#endif
