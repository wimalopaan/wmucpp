/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#define USE_MCU_STM_V3
#define USE_CRSF_V3

#define SERIAL_DEBUG // enable debug on esc-tlm-1

#define SW_VERSION 1
#define HW_VERSION 1

#define USE_RC720

#define NDEBUG

#include <cstdint>
#include <chrono>

#include "devices.h"
#include "gfsm.h"

using namespace std::literals::chrono_literals;

struct Storage {
    static inline void init() {
        std::memcpy(&eeprom, &eeprom_flash, sizeof(EEProm));
        // eeprom = eeprom_flash; // not working: needs volatile
    }
    static inline void reset() {
        eeprom = EEProm{};
    }
    __attribute__((__section__(".eeprom")))
    static inline const EEProm eeprom_flash;
    __attribute__ ((aligned (8)))
    static inline EEProm eeprom;
};

struct DevsConfig {
    using storage = Storage;
};
using devs = Devices<SW01, DevsConfig, Mcu::Stm::Stm32G0B1>;
using gfsm = GFSM<devs>;

int main() {
    Storage::init();
    gfsm::init();

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART2_LPUART2_IRQn);
    // NVIC_EnableIRQ(USART3_4_5_6_LPUART1_IRQn);
    // NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    // NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);
    // NVIC_EnableIRQ(ADC1_COMP_IRQn);
    // NVIC_EnableIRQ(TIM3_TIM4_IRQn);
    __enable_irq();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}

extern "C" {
void USART1_IRQHandler() {
    using crsf_in = devs::crsf_in;
    static_assert(crsf_in::number == 1);
    crsf_in::Isr::onIdle([]{
    });
    crsf_in::Isr::onTransferComplete([]{
    });
}
void USART2_LPUART2_IRQHandler(){
    using radio = devs::radio;
    static_assert(radio::uart::number == 2);
    radio::Isr::onTransferComplete([]{
    });
    radio::Isr::onIdle([]{
    });
}

extern int _end;
static unsigned char *heap = NULL;
void* _sbrk(const int incr) {
    if (heap == NULL) {
        heap = (unsigned char *)&_end;
    }
    unsigned char* prev_heap = heap;
    heap += incr;
    return prev_heap;
}
void _exit(int) {
    __asm("BKPT #0");
}
void _kill(int, int) {
    return;
}
int _getpid(void) {
    return -1;
}

}

