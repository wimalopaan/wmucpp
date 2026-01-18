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

#define NDEBUG

#define SERIAL_DEBUG
// #define USE_SWD

#define SW_VERSION 2
#define HW_VERSION 1

#include <cstdint>
#include <chrono>

#include "devices.h"
#include "gfsm.h"
#include "port_aux1.h"
#include "port_aux2.h"
#include "port_sm1.h"
#include "port_sm2.h"

using namespace std::literals::chrono_literals;

struct Storage {
    static inline void init() {
        std::memcpy(&eeprom, &eeprom_flash, sizeof(EEProm));
    }
    static inline void reset() {
        eeprom = EEProm{};
    }
    __attribute__((__section__(".eeprom"))) static inline const EEProm eeprom_flash;
    __attribute__ ((aligned (8))) static inline EEProm eeprom;
};

struct DevsConfig;
using devs = Devices<Desk01, DevsConfig, Mcu::Stm::Stm32G0B1>;
using gfsm = GFSM<devs>;

struct DevsConfig {
    using storage = Storage;
    using auxes1 = Auxes1<devs>;
    using auxes2 = Auxes2<devs>;
    using smes1 = Smes1<devs>;
    using smes2 = Smes2<devs>;
    using gfsm = ::gfsm;
};


int main() {
    Storage::init();
    gfsm::init();
    gfsm::updateFromEeprom();

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART2_LPUART2_IRQn);
    NVIC_EnableIRQ(USART3_4_5_6_LPUART1_IRQn);
    // NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    // NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);
    NVIC_EnableIRQ(ADC1_COMP_IRQn);
    // NVIC_EnableIRQ(TIM3_TIM4_IRQn);
    __enable_irq();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([] static {
            gfsm::ratePeriodic();
        });
    }
}

extern "C" {
void ADC1_COMP_IRQHandler() {
    using adc = devs::adc;
    static_assert(adc::number == 1);
    adc::Isr::onEnd([] static {
                        devs::tp::toggle();
                    });
}
void USART1_IRQHandler() {
    using bt = devs::bt;
    static_assert(bt::uart::number == 1);
    bt::Isr::onIdle([] static {});
    bt::Isr::onTransferComplete([] static {});
}

void USART3_4_5_6_LPUART1_IRQHandler() {
    using crsf_in = devs::crsf_in;
    static_assert(crsf_in::number == 6);
    crsf_in::Isr::onIdle([] static {});
    crsf_in::Isr::onTransferComplete([] static {});

    using sbus2 = devs::sbus2;
    static_assert(sbus2::uart::number == 3);
    sbus2::Isr::onIdle([] static {});
    sbus2::Isr::onTransferComplete([] static {});

    using hwext2 = devs::hwext2;
    static_assert(hwext2::uart::number == 3);
    hwext2::Isr::onIdle([] static {});
    hwext2::Isr::onTransferComplete([] static {});

    using sm1 = devs::sm1;
    static_assert(sm1::uart::number == 5);
    sm1::Isr::onIdle([] static {});
    sm1::Isr::onTransferComplete([] static {});

    using sm2 = devs::sm2;
    static_assert(sm2::uart::number == 4);
    sm2::Isr::onIdle([] static {});
    sm2::Isr::onTransferComplete([] static {});

}
void USART2_LPUART2_IRQHandler(){
#ifndef USE_SWD
    using sbus1 = devs::sbus1;
    static_assert(sbus1::uart::number == 2);
    sbus1::Isr::onIdle([] static {});
    sbus1::Isr::onTransferComplete([] static {});

    using hwext1 = devs::hwext1;
    static_assert(hwext1::uart::number == 2);
    hwext1::Isr::onTransferComplete([] static {});
    hwext1::Isr::onIdle([] static {});
#endif
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

#ifndef NDEBUG
void _close(int){
}
void _lseek(int){
}
void _read(int){
}
void _write(int){
}
void _fstat(int){
}
void _isatty(int){
}

#endif
}

