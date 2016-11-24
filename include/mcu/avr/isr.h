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

#include <stdint.h>
#include "util/bits.h"

#if __has_include(<avr/interrupt.h>)
# include <avr/interrupt.h>

extern "C" {
    void TIMER0_OVF_vect(void);
    void TIMER0_COMPA_vect(void);

    void USART_RX_vect(void);
    void USART_UDRE_vect(void);

    void USART0_RX_vect(void);
    void USART0_UDRE_vect(void);
    void USART1_RX_vect(void);
    void USART1_UDRE_vect(void);

    void SPI_STC_vect(void);

    void TIMER1_COMPA_vect(void);
    void TIMER1_COMPB_vect(void);
    void TIMER1_CAPT_vect(void);

    void TIMER3_COMPA_vect(void);
    void TIMER3_COMPB_vect(void);
    void TIMER3_CAPT_vect(void);

    void PCINT0_vect(void);
    void PCINT1_vect(void);
    void PCINT2_vect(void);
    void PCINT3_vect(void);

}

#endif

template<typename... HH>
struct IsrRegistrar {
    typedef uint64_t mask_type;
    
    static_assert(sizeof...(HH) <= 64, "too much different interrupts");
    
    static constexpr mask_type all = (HH::mask | ...);
    
    static void init() {
        static_assert(Util::numberOfOnes(all) == sizeof...(HH), "Isr double defined");
    }
    template<uint8_t In, uint8_t N, typename H, typename... Hp>
    struct Caller {
        static constexpr void call() {
            if (In == H::number) {
                H::isr();
            }
            if constexpr((N-1) > 0) {
                Caller<In, N-1, Hp..., void>::call();
            }
        }
    };
    
    template<typename INT>
    static void isr() {
        static_assert(all & ((mask_type)1 << INT::number), "isr not set");
        Caller<INT::number, sizeof...(HH), HH...>::call();
    }
};

template<typename I>
struct IsrBaseHandler {
    typedef I isr_type;
    static constexpr const uint8_t number = I::number;
    static constexpr const uint64_t mask = ((uint64_t)1 << I::number);    
};
template<>
struct IsrBaseHandler<void> {
    typedef void isr_type;
    static constexpr const uint8_t number = 0;
    static constexpr const uint64_t mask = 0;    
};

template<typename I, typename... Hs>
struct IsrDistributor final : public IsrBaseHandler<I> {
    static_assert(Util::numberOfOnes((Hs::mask | ...)) == 1, "all sub-handler must use same isr");
    static void isr() {
        (Hs::isr(), ...);
    }
};

namespace AVR {
namespace ISR {

template<uint8_t N>
struct Int;

template<>
struct Int<0> {
    static constexpr const uint32_t number = INT0_vect_num;
};
template<>
struct Int<1> {
    static constexpr const uint32_t number = INT1_vect_num;
};
#ifdef INT2_vect_num
template<>
struct Int<2> {
    static constexpr const uint32_t number = INT2_vect_num;
};
#endif

template<uint8_t N>
struct PcInt;

#ifdef PCINT0_vect_num
template<>
struct PcInt<0>{
    static constexpr const uint32_t number = PCINT0_vect_num;
};
#endif
#ifdef PCINT1_vect_num
template<>
struct PcInt<1>  {
    static constexpr const uint32_t number = PCINT1_vect_num;
};
#endif
#ifdef PCINT2_vect_num
template<>
struct PcInt<2>  {
    static constexpr const uint32_t number = PCINT2_vect_num;
};
#endif
#ifdef PCINT3_vect_num
template<>
struct PcInt<3>  {
    static constexpr const uint32_t number = PCINT3_vect_num;
};
#endif
#ifdef WDT_vect_num
struct Wdt  {
    static constexpr const uint32_t number = WDT_vect_num;
};
#endif

template<uint8_t N>
struct Timer;

template<>
struct Timer<2> {
    struct CompareA  {
#ifdef TIMER2_COMPA_vect_num
        static constexpr const uint32_t number = TIMER2_COMPA_vect_num;
#endif
    };
    struct CompareB  {
#ifdef TIMER2_COMPB_vect_num
        static constexpr const uint32_t number = TIMER2_COMPB_vect_num;
#endif
    };
    struct Compare  {
#ifdef TIMER2_COMP_vect_num
        static constexpr const uint32_t number = TIMER2_COMP_vect_num;
#endif
    };
    struct Overflow  {
        static constexpr const uint32_t number = TIMER2_OVF_vect_num;
    };
};
template<>
struct Timer<1> {
    struct Capture  {
        static constexpr const uint32_t number = TIMER1_CAPT_vect_num;
    };
    struct CompareA  {
        static constexpr const uint32_t number = TIMER1_COMPA_vect_num;
    };
    struct CompareB  {
        static constexpr const uint32_t number = TIMER1_COMPB_vect_num;
    };
    struct Overflow  {
        static constexpr const uint32_t number = TIMER1_OVF_vect_num;
    };
};
template<>
struct Timer<0> {
    struct CompareA  {
#ifdef TIMER0_COMPA_vect_num
        static constexpr const uint32_t number = TIMER0_COMPA_vect_num;
#endif
    };
    struct CompareB  {
#ifdef TIMER0_COMPB_vect_num
        static constexpr const uint32_t number = TIMER0_COMPB_vect_num;
#endif
    };
    struct Overflow  {
        static constexpr const uint32_t number = TIMER0_OVF_vect_num;
    };
};
struct SpiStc  {
    static constexpr const uint32_t number = SPI_STC_vect_num;
};


}
}