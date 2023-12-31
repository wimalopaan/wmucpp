/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <cstdint>
#include <type_traits>

#if __has_include(<avr/io.h>)
# include <avr/io.h>
# undef _MMIO_BYTE
# define _MMIO_BYTE(adr) ((uintptr_t)(adr))
#endif

#include "mcu/mcu.h"
#include "mcu/concepts.h"

namespace MCU {
    template<typename L>
    concept bool Letter() {
        return requires(L l) {
            L::letter;
        };
    }
    
    template<typename C>
    concept bool SingleComponent() {
        return requires(C c) {
            C::address;
        };
    }
    
    template<typename C>
    concept bool MultipleComponent() {
        return requires(C c) {
            C::count;
            //        C::template Address<0>::value; // not possible beacuse of timer numbering scheme 
        };
    }
}

namespace AVR {
    struct Inverting;
    struct NonInverting;
    
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
        return std::is_same<T, ATMega328P>::value || std::is_same<T, ATMega328PB>::value || 
                std::is_same<T, ATMega88P>::value || std::is_same<T, ATMega168P>::value;
    }
    template<typename T>
    concept bool ATMega_X4() {
        return std::is_same<T, ATMega1284P>::value || std::is_same<T, ATMega324PB>::value;
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

    inline static constexpr uintptr_t ioOffset = __SFR_OFFSET;
    
    template<MCU::MultipleComponent Component, int Number>
    constexpr inline bool isSBICBICapable() {
        return Component::template Address<Number>::value < (0x20 + ioOffset);
    }
    template<typename Component, MCU::Letter Name>
    constexpr inline bool isSBICBICapable() {
        return Component::template Address<Name>::value < (0x20 + ioOffset);
    }
    
    template<typename Component, MCU::Letter Letter>
    constexpr inline Component* getBaseAddr() {
        return reinterpret_cast<Component*>(Component::template Address<Letter>::value);
    }

} // !AVR


#include "register.h"

#if defined(__AVR_ATmega1284P__)
# include "mcu/avr/atmega1284p.h"
#elif defined(__AVR_ATmega328P__)
# include "mcu/avr/atmega328p.h"
#elif defined(__AVR_ATmega88P__)
# include "mcu/avr/atmega88p.h"
#elif defined(__AVR_ATmega168P__)
# include "mcu/avr/atmega168p.h"
#elif defined(__AVR_ATmega328PB__)
#include "mcu/avr/atmega328pb.h"
#elif defined(__AVR_ATmega8__)
#include "mcu/avr/atmega8.h"
#elif defined(__AVR_ATmega324PB__)
#include "mcu/avr/atmega324pb.h"
#elif defined(__AVR_ATtiny85__)
#include "mcu/avr/attiny85.h"
#elif defined(__AVR_ATtiny25__)
#include "mcu/avr/attiny25.h"
#elif defined(__AVR_ATtiny84__)
#include "mcu/avr/attiny84.h"
#else
typedef AVR::ATMegaNone DefaultMcuType;
#endif

#undef _MMIO_BYTE
#define _MMIO_BYTE(addr) (*(volatile uint8_t*)(addr))
