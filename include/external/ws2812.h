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

#include <util/delay.h>
#include <avr/interrupt.h>

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "util/dassert.h"
#include "util/disable.h"
#include "std/array"
#include "units/percent.h"

struct ColorSequenceRGB {};
struct ColorSequenceGRB {};

struct Red {
    uint8_t value = 0;  
};
struct Green {
    uint8_t value = 0;  
};
struct Blue {
    uint8_t value = 0;  
};

template<typename C = ColorSequenceRGB>
struct cRGB;

template<>
struct cRGB<ColorSequenceRGB> {
    static constexpr cRGB createFrom(const std::array<uint8_t, 3> bytes) {
        return {Red{bytes[0]}, Green{bytes[1]}, Blue{bytes[2]}};
    }

    constexpr cRGB() {}
    cRGB(const volatile cRGB& o) : r(o.r), g(o.g), b(o.b) {}
    constexpr cRGB(const cRGB& o) : r(o.r), g(o.g), b(o.b) {}
    constexpr explicit cRGB(uint8_t v) : r(v), g(v), b(v) {}
    constexpr cRGB(Red red, Green green, Blue blue) : r(red.value), g(green.value), b(blue.value) {}
    constexpr cRGB(Red v) : r(v.value) {}
    constexpr cRGB(Green v) : g(v.value) {}
    constexpr cRGB(Blue v) : b(v.value) {}
    constexpr cRGB& operator+=(const cRGB& c) {
        if (r > 0) {
            r = (r + c.r) / 2;
        }
        else {
            r = c.r;
        }
        if (g > 0) {
            g = (g + c.g) / 2;
        }
        else {
            g = c.g;
        }
        if (b > 0) {
            b = (b + c.b) / 2;
        }
        else {
            b = c.b;
        }
        return *this;
    }
    constexpr cRGB& operator*=(const std::percent& p) {
        g = std::expand(p, uint8_t(0), g);
        r = std::expand(p, uint8_t(0), r);
        b = std::expand(p, uint8_t(0), b);
        return *this;        
    }

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
};

constexpr cRGB<ColorSequenceRGB> operator*(cRGB<ColorSequenceRGB> c, const std::percent& p) {
    return c *= p;
}

template<>
struct cRGB<ColorSequenceGRB> {
    static constexpr cRGB createFrom(const std::array<std::byte, 3> bytes) {
        return {Red{std::to_integer<uint8_t>(bytes[0])}, 
            Green{std::to_integer<uint8_t>(bytes[1])}, 
            Blue{std::to_integer<uint8_t>(bytes[2])}};
    }
    constexpr cRGB() {}
    cRGB(const volatile cRGB& o) : g(o.g), r(o.r), b(o.b) {}
    constexpr cRGB(const cRGB& o) : g(o.g), r(o.r), b(o.b) {}
    constexpr explicit cRGB(uint8_t v) : g(v), r(v), b(v) {}
    constexpr cRGB(Red red, Green green, Blue blue) : g(green.value), r(red.value), b(blue.value) {}
    constexpr cRGB(Red v) : r(v.value) {}
    constexpr cRGB(Green v) : g(v.value) {}
    constexpr cRGB(Blue v) : b(v.value) {}
    constexpr cRGB& operator+=(const cRGB& c) {
        if (r > 0) {
            r = (r + c.r) / 2;
        }
        else {
            r = c.r;
        }
        if (g > 0) {
            g = (g + c.g) / 2;
        }
        else {
            g = c.g;
        }
        if (b > 0) {
            b = (b + c.b) / 2;
        }
        else {
            b = c.b;
        }
        return *this;
    }
    constexpr cRGB& operator*=(const std::percent& p) {
        g = std::expand(p, uint8_t(0), g);
        r = std::expand(p, uint8_t(0), r);
        b = std::expand(p, uint8_t(0), b);
        return *this;        
    }
    uint8_t g = 0;
    uint8_t r = 0;
    uint8_t b = 0;
};

constexpr cRGB<ColorSequenceGRB> operator*(cRGB<ColorSequenceGRB> c, const std::percent& p) {
    return c *= p;
}

template<uint8_t N, MCU::Pin Pin, typename ColorComp = ColorSequenceRGB, bool DisableInterruptsFull = false>
class WS2812 final {
    WS2812() = delete;
public:
    static constexpr uint8_t size = N;
    typedef Pin pin_type;
    typedef cRGB<ColorComp> item_type;
    
    typedef ColorComp color_sequence;
    typedef cRGB<ColorComp> color_type;
    
    static void init() {
        Pin::template dir<AVR::Output>();
    }
    static void off() {
        set(cRGB<ColorComp>());
    }
    template<bool writeOut = true>
    static void set(uint8_t number, const item_type& color) {
        assert(number < N);
        leds[number] = color;
        if constexpr(writeOut) {
            write();
        }
    }
    template<bool writeOut = true>
    static void add(uint8_t number, const item_type& color) {
        assert(number < N);
        leds[number] += color;
        if constexpr(writeOut) {
            write();
        }
    }
    template<bool writeOut = true>
    static void set(const item_type& color) {
        for(auto& l : leds) {
            l = color;
        }
        if constexpr(writeOut) {
            write();
        }
    }
    static constexpr item_type& elementAt(uint8_t index) {
        assert(index < N);
        return leds[index];
    }
private:
    inline static std::array<item_type, N> leds;
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
        
        std::byte masklo = ~Pin::pinMask & Pin::port::get();
        std::byte maskhi = Pin::pinMask | Pin::port::get();
        
        //        uint8_t sreg_prev = SREG;
        //        cli();
        
        Scoped<DisbaleInterrupt<RestoreState>, DisableInterruptsFull> di;
        
        for(const uint8_t* data = reinterpret_cast<uint8_t*>(&leds[0]); data < reinterpret_cast<uint8_t*>(&leds[N - 1] + 1); ++data) {
            {
                Scoped<DisbaleInterrupt<RestoreState>, !DisableInterruptsFull> di;
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
        }
        //        SREG=sreg_prev;
    }
};
