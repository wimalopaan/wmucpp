#define USE_MCU_STM_V3
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

// #define USE_CRSF_V3
#define ESCAPE32_ASCII // enable ESCape32 ascii configuration parameter menu
// #define ESCAPE32_U8 // use only 8bit capable parameters
#define SERVO_CALIBRATION // enable analog feedback servo calibration
#define SERVO_ADDRESS_SET // eanble setting waveshare servo IDs
#define CRSF_ADDRESS 192
#define SERIAL_DEBUG // enable debug on esc-tlm-1
#define TEST_EEPROM // fill eeprom with test setup

#define SW_VERSION 13
#define HW_VERSION 2

#define NDEBUG

#include <cstdint>
#include <chrono>

#include "output_aux.h"
#include "output_esc.h"
#include "output_relay.h"
#include "output_servo.h"
#include "gfsm_2.h"
#include "devices_2.h"

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

struct DevsConfig;
using devs = Devices<SW01, DevsConfig, Mcu::Stm::Stm32G0B1>;
using servooutputs = ServoOutputs<devs>;
using escoutputs = EscOutputs<devs>;
using relayoutputs = Relays<devs>;
using auxoutputs = Auxes<devs>;

using gfsm = GFSM<devs, servooutputs, escoutputs, relayoutputs, auxoutputs>;

struct DevsConfig {
    using storage = Storage;
    using servos = servooutputs;
    using escs = escoutputs;
    using relays = relayoutputs;
    using auxes = auxoutputs;
    using compass = gfsm::compass;
};

int main() {
    Storage::init();
    gfsm::init();
    gfsm::updateFromEeprom();

    static constexpr uint8_t defaultIntPrio = 1;
    static constexpr uint8_t swUartIntPrio  = 0;

    static_assert(swUartIntPrio < defaultIntPrio);

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, defaultIntPrio);

    NVIC_EnableIRQ(USART2_LPUART2_IRQn);
    NVIC_SetPriority(USART2_LPUART2_IRQn, defaultIntPrio);

    NVIC_EnableIRQ(USART3_4_5_6_LPUART1_IRQn);
    NVIC_SetPriority(USART3_4_5_6_LPUART1_IRQn, defaultIntPrio);

    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    NVIC_SetPriority(DMA1_Channel2_3_IRQn, defaultIntPrio);

    // NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);
    // NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn, defaultIntPrio);

    NVIC_EnableIRQ(ADC1_COMP_IRQn);
    NVIC_SetPriority(ADC1_COMP_IRQn, defaultIntPrio);

    NVIC_EnableIRQ(TIM3_TIM4_IRQn);
    NVIC_SetPriority(TIM3_TIM4_IRQn, defaultIntPrio);

    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, swUartIntPrio);

    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_SetPriority(EXTI0_1_IRQn, swUartIntPrio);

    __enable_irq();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}
extern "C" {

// software-uart
void TIM1_BRK_UP_TRG_COM_IRQHandler() {
    using sbus_aux = devs::sbus_aux;
    using swuart = sbus_aux::uart;
    swuart::Isr::period();
}
// software-uart
void EXTI0_1_IRQHandler() {
    using sbus_aux = devs::sbus_aux;
    using swuart = sbus_aux::uart;
    swuart::Isr::edge();
}

void TIM3_TIM4_IRQHandler() {
    using pulse_in = devs::pulse_in;
    static_assert(pulse_in::timerNumber == 4);
    pulse_in::onCapture([]{
    });
}

void ADC1_COMP_IRQHandler() {
    using adc = devs::adc;
    if (adc::mcuAdc->ISR & ADC_ISR_EOS) {
        adc::mcuAdc->ISR = ADC_ISR_EOS; // end-of-sequence
    }
}
void DMA1_Channel2_3_IRQHandler() {
    using adc = devs::adc;
    using adcDma = adc::dmaChannel;
    static_assert(adcDma::number == 3);
    adcDma::onTransferComplete([]{
    });
}
// void DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQHandler() {
// }
void USART2_LPUART2_IRQHandler(){
    {
        using esc32_1 = devs::esc32_1;
        static_assert(esc32_1::uart::number == 2);
        esc32_1::Isr::onTransferComplete([]{
        });
        esc32_1::Isr::onIdle([]{
        });
        using esc32ascii_1 = devs::esc32ascii_1;
        static_assert(esc32ascii_1::uart::number == 2);
        esc32ascii_1::Isr::onTransferComplete([]{
        });
        esc32ascii_1::Isr::onIdle([]{
        });
        using vesc_1 = devs::vesc_1;
        static_assert(vesc_1::uart::number == 2);
        vesc_1::Isr::onTransferComplete([]{
        });
        vesc_1::Isr::onIdle([]{
        });
        esc32_1::uart::mcuUart->ICR = -1;
    }
    {
        using sbus1 = devs::sbus1;
        static_assert(sbus1::uart::number == 102);
        sbus1::Isr::onTransferComplete([]{
        });
        sbus1::Isr::onIdle([]{
        });
        using relay = devs::relay1;
        static_assert(relay::uart::number == 102);
        relay::Isr::onTransferComplete([]{
        });
        relay::Isr::onIdle([]{
        });
        using ibus_in = devs::ibus_in;
        static_assert(ibus_in::uart::number == 102);
        ibus_in::Isr::onIdle([]{
        });
        using sbus_in = devs::sbus_in;
        static_assert(sbus_in::uart::number == 102);
        sbus_in::Isr::onIdle([]{
        });
        using sumdv3_in = devs::sumdv3_in;
        static_assert(sumdv3_in::uart::number == 102);
        sumdv3_in::Isr::onIdle([]{
        });
        // sbus1 / relay1 / ibus / sbus_in / sumdv3 use same LPUART(2), be sure to clear all flags
        relay::uart::mcuUart->ICR = -1;
    }
}
void USART3_4_5_6_LPUART1_IRQHandler(){
    {
        using esc32_2 = devs::esc32_2;
        static_assert(esc32_2::uart::number == 3);
        esc32_2::Isr::onTransferComplete([]{
        });
        esc32_2::Isr::onIdle([]{
        });
        using esc32ascii_2 = devs::esc32ascii_2;
        static_assert(esc32ascii_2::uart::number == 3);
        esc32ascii_2::Isr::onTransferComplete([]{
        });
        esc32ascii_2::Isr::onIdle([]{
        });
        using vesc_2 = devs::vesc_2;
        static_assert(vesc_2::uart::number == 3);
        vesc_2::Isr::onTransferComplete([]{});
        vesc_2::Isr::onIdle([]{});

        esc32_2::uart::mcuUart->ICR = -1;
    }
    {
        using ws1 = devs::srv1_waveshare;
        static_assert(ws1::uart::number == 5);
        ws1::Isr::onTransferComplete([]{
        });
        ws1::Isr::onIdle([]{
        });
    }
    {
        using ws2 = devs::srv2_waveshare;
        static_assert(ws2::uart::number == 6);
        ws2::Isr::onTransferComplete([]{
        });
        ws2::Isr::onIdle([]{
        });
    }
    {
        using relay = devs::relay_aux;
        static_assert(relay::uart::number == 4);
        relay::Isr::onTransferComplete([]{
        });
        relay::Isr::onIdle([]{
        });
        using sport = devs::sport_aux;
        static_assert(sport::uart::number == 4);
        sport::Isr::onTransferComplete([]{
        });
        sport::Isr::onIdle([]{
        });
        using gps = devs::gps_aux;
        static_assert(gps::uart::number == 4);
        // gps::Isr::onTransferComplete([]{
        // });
        gps::Isr::onIdle([]{
        });
        using bt = devs::bt;
        static_assert(bt::uart::number == 4);
        bt::Isr::onTransferComplete([]{
        });
        bt::Isr::onIdle([]{
        });
    }
}
void USART1_IRQHandler() {
    using crsf_in = devs::crsf_in;
    static_assert(crsf_in::number == 1);
    crsf_in::Isr::onIdle([]{
    });
    crsf_in::Isr::onTransferComplete([]{
        // devs::tp3::set();
        // devs::tp3::reset();
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

// strtof()
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

