#pragma once

#include <type_traits>
#include <concepts>

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "units.h"
#include "concepts.h"
#include "rcc.h"
#include "atomic.h"

template<typename Config>
struct WatchDog {
    static inline constexpr uint32_t reload = Config::reload;
    static inline void init() {
        if (RCC->CSR & RCC_CSR_PWRRSTF) {
            pin_count = 0;
            wdg_count = 0;
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
private:
    static inline void reset() {
        IWDG->KR = 0x0000aaaa;
    }
    __attribute__((__section__(".noinit")))
    static inline volatile uint8_t pin_count;
    __attribute__((__section__(".noinit")))
    static inline volatile uint8_t wdg_count;
};
