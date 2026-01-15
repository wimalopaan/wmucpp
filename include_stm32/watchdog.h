/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <type_traits>
#include <concepts>

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "units.h"
#include "concepts.h"
#include "rcc.h"

// avoids linker warning
__attribute__((__section__(".noinit")))
static inline volatile uint8_t pin_count;
__attribute__((__section__(".noinit")))
static inline volatile uint8_t wdg_count;
__attribute__((__section__(".noinit")))
static inline volatile uint32_t test_count;

template<typename Config>
struct WatchDog {
    static inline constexpr uint32_t reload = Config::reload;
    static inline void init() {
        if (RCC->CSR & RCC_CSR_PWRRSTF) {
            pin_count = 0;
            wdg_count = 0;
            test_count = 0;
            RCC->CSR = RCC_CSR_RMVF;
        }
        else {
            if (RCC->CSR & RCC_CSR_IWDGRSTF) {
                wdg_count = wdg_count + 1;
            }
            if (RCC->CSR & RCC_CSR_PINRSTF) {
                pin_count = pin_count + 1;
            }
            RCC->CSR = RCC_CSR_RMVF;
        }

        RCC->CSR |= RCC_CSR_LSION;
        while(!(RCC->CSR & RCC_CSR_LSIRDY));

        IWDG->KR = 0x0000cccc;
        IWDG->KR = 0x00005555;

        while(IWDG->SR & IWDG_SR_PVU);
        IWDG->PR = 0b011; // div 32 = 1KHz

        while(IWDG->SR & IWDG_SR_RVU);
        IWDG->RLR = reload;

        while(IWDG->SR & 0b111);

        IWDG->KR = 0x0000aaaa;
    }
    static inline void ratePeriodic() {
        reset();
    }
    static inline uint8_t pinResets() {
        return pin_count;
    }
    static inline uint8_t wdgResets() {
        return wdg_count;
    }
    static inline uint32_t testCount() {
        return test_count;
    }
    static inline void testLoop() {
        test_count = 0;
        while(true) {
            test_count = test_count + 1;
        }
    }
private:
    static inline void reset() {
        IWDG->KR = 0x0000aaaa;
    }
    // produces linker warning: SHF_GROUP
    // __attribute__((__section__(".noinit")))
    // static inline volatile uint8_t pin_count;
    // __attribute__((__section__(".noinit")))
    // static inline volatile uint8_t wdg_count;
};
