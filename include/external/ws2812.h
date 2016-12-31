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

#include <util/delay.h>
#include <avr/interrupt.h>

#include "config.h"
#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "util/dassert.h"
#include "std/array.h"

struct cRGB {
    uint8_t g = 0;
    uint8_t r = 0;
    uint8_t b = 0;
};

template<uint8_t N, typename Pin>
class WS2812 final {
public:
    static constexpr uint8_t size = N;
    typedef Pin pin_type;
    typedef cRGB item_type;
    
    WS2812() = delete;
    static void init() {
        Pin::template dir<AVR::Output>();
    }
    static void off() {
        set(cRGB());
    }
    template<bool writeOut = true>
    static void set(uint8_t number, const cRGB& color) {
        assert(number < N);
        leds[number] = color;
        if constexpr(writeOut) {
            write();
        }
    }
    template<bool writeOut = true>
    static void set(const cRGB& color) {
        for(auto& l : leds) {
            l = color;
        }
        if constexpr(writeOut) {
            write();
        }
    }
    static constexpr cRGB& elementAt(uint8_t index) {
        assert(index < N);
        return leds[index];
    }
private:
    static std::array<cRGB, N> leds;
public:
    static void write() {
    // Timing in ns
#define w_zeropulse   350
#define w_onepulse    900
#define w_totalperiod 1250
    //#define w_onepulse    1350
    //#define w_totalperiod 1700

    // Fixed cycles used by the inner loop
#define w_fixedlow    2
#define w_fixedhigh   4
#define w_fixedtotal  8

    // Insert NOPs to match the timing, if possible
#define w_zerocycles    (((F_CPU/1000)*w_zeropulse          )/1000000)
#define w_onecycles     (((F_CPU/1000)*w_onepulse    +500000)/1000000)
#define w_totalcycles   (((F_CPU/1000)*w_totalperiod +500000)/1000000)

    // w1 - nops between rising edge and falling edge - low
#define w1 (w_zerocycles-w_fixedlow)
    // w2   nops between fe low and fe high
#define w2 (w_onecycles-w_fixedhigh-w1)
    // w3   nops to complete loop
#define w3 (w_totalcycles-w_fixedtotal-w1-w2)

#if w1>0
#define w1_nops w1
#else
#define w1_nops  0
#endif

    // The only critical timing parameter is the minimum pulse length of the "0"
    // Warn or throw error if this timing can not be met with current F_CPU settings.
#define w_lowtime ((w1_nops+w_fixedlow)*1000000)/(F_CPU/1000)
#if w_lowtime>550
#error "Light_ws2812: Sorry, the clock speed is too low. Did you set F_CPU correctly?"
#elif w_lowtime>450
#warning "Light_ws2812: The timing is critical and may only work on WS2812B, not on WS2812(S)."
#warning "Please consider a higher clockspeed, if possible"
#endif

#if w2>0
#define w2_nops w2
#else
#define w2_nops  0
#endif

#if w3>0
#define w3_nops w3
#else
#define w3_nops  0
#endif

#define w_nop1  "nop      \n\t"
#define w_nop2  "rjmp .+0 \n\t"
#define w_nop4  w_nop2 w_nop2
#define w_nop8  w_nop4 w_nop4
#define w_nop16 w_nop8 w_nop8

        uint8_t ctr;

        uint8_t masklo = ~Pin::pinMask  & Pin::port::get();
        uint8_t maskhi = Pin::pinMask | Pin::port::get();

        uint8_t sreg_prev = SREG;
        cli();

        for(const uint8_t* data = reinterpret_cast<uint8_t*>(&leds[0]); data < reinterpret_cast<uint8_t*>(&leds[N - 1] + 1); ++data) {
            asm volatile(
                        "       ldi   %0,8  \n\t"
                        "loop%=:            \n\t"
                        "       out   %2,%3 \n\t"    //  '1' [01] '0' [01] - re
            #if (w1_nops&1)
                        w_nop1
            #endif
            #if (w1_nops&2)
                        w_nop2
            #endif
            #if (w1_nops&4)
                        w_nop4
            #endif
            #if (w1_nops&8)
                        w_nop8
            #endif
            #if (w1_nops&16)
                        w_nop16
            #endif
                        "       sbrs  %1,7  \n\t"    //  '1' [03] '0' [02]
                        "       out   %2,%4 \n\t"    //  '1' [--] '0' [03] - fe-low
                        "       lsl   %1    \n\t"    //  '1' [04] '0' [04]
            #if (w2_nops&1)
                        w_nop1
            #endif
            #if (w2_nops&2)
                        w_nop2
            #endif
            #if (w2_nops&4)
                        w_nop4
            #endif
            #if (w2_nops&8)
                        w_nop8
            #endif
            #if (w2_nops&16)
                        w_nop16
            #endif
                        "       out   %2,%4 \n\t"    //  '1' [+1] '0' [+1] - fe-high
            #if (w3_nops&1)
                        w_nop1
            #endif
            #if (w3_nops&2)
                        w_nop2
            #endif
            #if (w3_nops&4)
                        w_nop4
            #endif
            #if (w3_nops&8)
                        w_nop8
            #endif
            #if (w3_nops&16)
                        w_nop16
            #endif

                        "       dec   %0    \n\t"    //  '1' [+2] '0' [+2]
                        "       brne  loop%=\n\t"    //  '1' [+3] '0' [+4]
                        :	"=&d" (ctr)
                        :	"r" (*data), "I" (Pin::port::address() - __SFR_OFFSET), "r" (maskhi), "r" (masklo)
                        );
        }
        SREG=sreg_prev;
    }
};

template<uint8_t N, typename Pin>
std::array<cRGB, N> WS2812<N, Pin>::leds;
