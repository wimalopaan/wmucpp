#pragma once

#if __has_include(<avr/io.h>)
# include <avr/io.h>
# undef _MMIO_BYTE
# define _MMIO_BYTE(adr) ((uintptr_t)(adr))
# define _MMIO_BYTE_CHANGED
#endif
#if __has_include(<util/twi.h>)
# include <util/twi.h>
#endif
#if __has_include(<avr/interrupt.h>)
# include <avr/interrupt.h>
#endif
#if __has_include(<avr/pgmspace.h>)
# include <avr/pgmspace.h>
#endif

#ifndef __AVR_ARCH__ 
# warning "__AVR_ARCH__ not defined"
#endif

#include "common/assert.h"
#include "../config.h"

#include <type_traits>

namespace AVR {
    //AVR Series
    struct ATMegaNone;
    struct ATMega8;
    struct ATMega88P;
    struct ATMega168P;
    struct ATMega1284P;
    struct ATMega328P;
    struct ATMega328PB;
    struct ATMega324PB;
    struct ATTiny84;
    struct ATTiny85;
    struct ATTiny25;
    //AVR0-Series
    struct ATMega4809;
}

#if defined(__AVR_ATmega1284P__)
typedef AVR::ATMega1284P DefaultMcuType;
#elif defined(__AVR_ATmega328P__)
typedef AVR::ATMega328P DefaultMcuType;
#elif defined(__AVR_ATmega88P__)
typedef AVR::ATMega88P DefaultMcuType;
#elif defined(__AVR_ATmega168P__)
typedef AVR::ATMega168P DefaultMcuType;
#elif defined(__AVR_ATmega328PB__)
typedef AVR::ATMega328PB DefaultMcuType;
#elif defined(__AVR_ATmega8__)
typedef AVR::ATMega8 DefaultMcuType;
#elif defined(__AVR_ATmega324PB__)
typedef AVR::ATMega324PB DefaultMcuType;
#elif defined(__AVR_ATtiny85__)
typedef AVR::ATTiny85 DefaultMcuType;
#elif defined(__AVR_ATtiny25__)
typedef AVR::ATTiny25 DefaultMcuType;
#elif defined(__AVR_ATtiny84__)
typedef AVR::ATTiny84 DefaultMcuType;
#elif defined(__SAMD21G18A__)
typedef ARM::SAM::SamD21 DefaultMcuType;
#elif defined(__AVR_ATmega4809__)
typedef AVR::ATMega4809 DefaultMcuType;
#else
typedef AVR::ATMegaNone DefaultMcuType;
#endif

#include "common/concepts.h"

namespace AVR {
    struct ReadWrite {};
    struct ReadOnly{};
    struct UnUsed{};
    
    template<bool F>
    struct UseInterrupts : std::integral_constant<bool, F> {};
    
    struct A : std::integral_constant<char, 'A'> {};
    struct B : std::integral_constant<char, 'B'> {};
    struct C : std::integral_constant<char, 'C'> {};
    struct D : std::integral_constant<char, 'D'> {};
    struct E : std::integral_constant<char, 'E'> {};
    
    template<AVR::Concepts::McuSingleComponent Component>
    constexpr inline Component* getBaseAddr() {
        return reinterpret_cast<Component*>(Component::address);
    }
    
    template<AVR::Concepts::McuMultipleComponent Component, int Number>
    constexpr inline Component* getBaseAddr() {
        return reinterpret_cast<Component*>(Component::template Address<Number>::value);
    }

    template<typename Component, AVR::Concepts::Letter Letter>
    constexpr inline Component* getBaseAddr() {
        return reinterpret_cast<Component*>(Component::template Address<Letter>::value);
    }

    inline static constexpr uintptr_t ioOffset = __SFR_OFFSET;
    
    template<typename Component, int Number>
    constexpr inline bool isSBICBICapable() {
        return Component::template Address<Number>::value < (0x20 + ioOffset);
    }
    template<typename Component, typename Name>
    constexpr inline bool isSBICBICapable() {
        return Component::template Address<Name>::value < (0x20 + ioOffset);
    }
}

namespace AVR::detail::test {
    using arch = std::integral_constant<int, __AVR_ARCH__>;
//    arch::_;
}

#if (__AVR_ARCH__ >= 2) && (__AVR_ARCH__ <= 51)
# include "mega/atmega.h"
# include "tiny/attiny.h"
# include "internals/port.h"
# include "internals/usart.h"
#endif

#if (__AVR_ARCH__ == 103)
# include "megaavr0/atxmega.h"
# include "internals/port.h"
# include "internals/usart.h"
#endif

#ifdef _MMIO_BYTE_CHANGED
# undef _MMIO_BYTE
# define _MMIO_BYTE(addr) (*(volatile uint8_t*)(addr))
#endif
