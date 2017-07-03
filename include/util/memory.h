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

namespace Util::Memory {
    
    extern "C" {
    extern uint8_t __heap_start;
    }
    
    static constexpr uint8_t testMask = 0xaa;
    
    void __attribute__ ((naked, used, section(".init3"))) initializeMemory() {
        __asm volatile (
                    "ldi r30, lo8 (__heap_start)"  "\n\t"
                    "ldi r31, hi8 (__heap_start)"  "\n\t"
                    "ldi r24, %0"                  "\n\t"
                    "ldi r25, hi8 (%1)"            "\n"
                    "0:"                           "\n\t"
                    "st  Z+,  r24"                 "\n\t"
                    "cpi r30, lo8 (%1)"            "\n\t"
                    "cpc r31, r25"                 "\n\t"
                    "brne 0b"
                    :
                    : "i" (testMask), "i" (RAMEND)
                    );
    }
    
    inline uint16_t getUnusedMemory() {
        uint8_t *p = &__heap_start;
        do {
            if (*p++ != testMask) {
                break;
            }
        } while (p <= (uint8_t*) RAMEND);
        return p - &__heap_start - 1;
    }
    
}
