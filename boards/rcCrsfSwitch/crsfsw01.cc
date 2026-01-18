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

#define SW_VERSION 2
#define HW_VERSION 1

#include <cstdint>
#include <chrono>

#include "devices.h"
#include "gfsm.h"

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

struct DevsConfig {
    using storage = Storage;
};

using devs = Devices<SW01, DevsConfig, Mcu::Stm::Stm32G0B1>;
using gfsm = GFSM<devs>;

int main() {
    gfsm::init();

    static constexpr uint8_t defaultIntPrio = 1;

//    static_assert(swUartIntPrio < defaultIntPrio);

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, defaultIntPrio);

    NVIC_EnableIRQ(USART2_LPUART2_IRQn);
    NVIC_SetPriority(USART2_LPUART2_IRQn, defaultIntPrio);

    NVIC_EnableIRQ(USART3_4_5_6_LPUART1_IRQn);
    NVIC_SetPriority(USART3_4_5_6_LPUART1_IRQn, defaultIntPrio);

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
    crsf_in::Isr::onIdle([]{});
    crsf_in::Isr::onTransferComplete([]{});
}
void USART2_LPUART2_IRQHandler() {
    using crsf_2 = devs::crsf_2;
    static_assert(crsf_2::uart::number == 2);
    crsf_2::Isr::onIdle([] static {});
    crsf_2::Isr::onTransferComplete([] static {});

    using crsf_7 = devs::crsf_7;
    static_assert(crsf_7::uart::number == 102);
    crsf_7::Isr::onIdle([] static {});
    crsf_7::Isr::onTransferComplete([] static {});
}
void USART3_4_5_6_LPUART1_IRQHandler() {
    using crsf_1 = devs::crsf_1;
    static_assert(crsf_1::uart::number == 101);
    crsf_1::Isr::onIdle([] static {});
    crsf_1::Isr::onTransferComplete([] static {});

    using crsf_3 = devs::crsf_3;
    static_assert(crsf_3::uart::number == 3);
    crsf_3::Isr::onIdle([] static {});
    crsf_3::Isr::onTransferComplete([] static {});

    using crsf_5 = devs::crsf_5;
    static_assert(crsf_5::uart::number == 5);
    crsf_5::Isr::onIdle([] static {});
    crsf_5::Isr::onTransferComplete([] static {});

    using crsf_6 = devs::crsf_6;
    static_assert(crsf_6::uart::number == 6);
    crsf_6::Isr::onIdle([] static {});
    crsf_6::Isr::onTransferComplete([] static {});
}


}

