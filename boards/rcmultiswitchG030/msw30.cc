/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

// select one of the following hardware definitions
// ATTENTION: use Makefile
//#define HW_MSW10 // MultiSwitch_10 (32K)
// ATTENTION: use Makefile.G031
#define HW_MSW11 // MultiSwitch_11 (64k)
//#define HW_NUCLEO // STM Nucleo G031K8 (64K) (incl. ST-Link)
// #define HW_WEACT // WeAct G031F8 (64K)

// #define USE_TP1 // enable test point
#define USE_MORSE
#define USE_OPERATE_MENU
#define USE_VIRTUALS
#define USE_PATTERNS
#define USE_TELEMETRY // switches telemetry default on (instead off)
// #define USE_BUTTON // HW_MSW11: if button is unused, the button pin is used as input (status bit)
// #define SERIAL_DEBUG // use with care (e.g. with USE_MORSE) because of RAM overflow
// #define CRSF_TX_OPENDRAIN // only HW_NUCLEO / HW_WEACT / HW_MSW11 : make tx pin open-drain to parallelize in two-wire mode
// #define CRSF_HALFDUPLEX // only NW_NUCLEO / HW_WEACT / HW_MSW11 : make crsf uart one-wire halfduplex (txpin), custom board is allways half-duplex
// #define USE_RESPONSE_SLOT // enables arbitration (slot after link-stat), important for half-duplex without crsf-switch/router

#define NDEBUG // do not change: dev option

#if defined(HW_MSW10)
# define HW_VERSION 1 // version of own pcb (not nucleo nor weact)
#elif defined(HW_MSW11)
# define HW_VERSION 2 // version 2 of own pcb (not nucleo nor weact)
#elif defined(HW_NUCLEO)
# define HW_VERSION 10
#elif defined(HW_WEACT)
# define HW_VERSION 11
# ifdef USE_BUTTON
#  warning "button on pin PA14 disables SWD programming -> use NRST to reset when start programming"
#  ifdef SERIAL_DEBUG
#   warning "button disabled (pin collision)"
#  endif
# endif
# ifdef SERIAL_DEBUG
#  warning "serial out pn PA14 disables SWD programming -> use NRST to reset when start programming"
# endif
#else
# error "wrong hardware definition"
#endif

#ifdef HW_MSW10
# define USE_RESPONSE_SLOT // enables arbitration (slot after link-stat), important for half-duplex without crsf-switch/router
# undef USE_VIRTUALS
# undef USE_PATTERNS
#endif

#define SW_VERSION 31

#include <cstdint>
#include <array>

#include "eeprom.h"
#include "devices_3.h"
#include "gfsm.h"

template<typename Config>
struct Storage {
    using debug = Config::debug;

    static inline void init() {
        std::memcpy(&eeprom, &eeprom_flash, sizeof(EEProm));
    }
    static inline void reset() {
        eeprom = EEProm{};
    }
    static inline void save() {
        if (const auto [ok, err] = Mcu::Stm32::savecfg(eeprom, eeprom_flash); ok) {
            IO::outl<debug>("# EEPROM OK");
        }
        else {
            IO::outl<debug>("# EEPROM NOK: ", err);
        }
    }
    __attribute__((__section__(".eeprom"))) static inline const EEProm eeprom_flash{};
    __attribute__ ((aligned (8))) static inline EEProm eeprom;
};

struct DevsConfig;

#ifdef HW_MSW10
using devs = Devices<SW20, DevsConfig>;
#endif
#ifdef HW_MSW11
using devs = Devices<SW21, DevsConfig>;
#endif
#ifdef HW_NUCLEO
using devs = Devices<Nucleo, DevsConfig>;
#endif
#ifdef HW_WEACT
using devs = Devices<WeAct, DevsConfig>;
#endif

struct DevsConfig {
    using storage = Storage<devs>;
};

int main() {
    using gfsm = GFSM<devs>;
    using storage = devs::storage;

    storage::init();
    gfsm::init();
    gfsm::update(true);

#ifdef HW_MSW10
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);
#endif
#if defined(HW_NUCLEO) || defined(HW_WEACT) || defined(HW_MSW11)
    NVIC_EnableIRQ(USART1_IRQn);
#endif
    NVIC_EnableIRQ(HardFault_IRQn);
    __enable_irq();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}

extern "C" {

#ifdef HW_MSW10
void USART2_IRQHandler(){
    using crsf = devs::crsf;
    static_assert(crsf::number == 2);
    crsf::Isr::onTransferComplete([]{});
    crsf::Isr::onIdle([]{});
}
#endif
#if defined(HW_NUCLEO) || defined(HW_WEACT) || defined(HW_MSW11)
void USART1_IRQHandler(){
    using crsf = devs::crsf;
    static_assert(crsf::number == 1);
    crsf::Isr::onTransferComplete([]{});
    crsf::Isr::onIdle([]{});
}
#endif
#ifdef HW_MSW10
void TIM3_IRQHandler() {
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR = ~TIM_SR_UIF;
        devs::bsw1::set();
        devs::bsw6::set();
    }
    if (TIM3->SR & TIM_SR_CC3IF) {
        TIM3->SR = ~TIM_SR_CC3IF;
        devs::bsw1::reset();
    }
    if (TIM3->SR & TIM_SR_CC4IF) {
        TIM3->SR = ~TIM_SR_CC4IF;
        devs::bsw6::reset();
    }
}
#endif

void HardFault_Handler() {
    while(true) {
#ifdef USE_TP1
        devs::tp1::set();
        devs::tp1::reset();
#endif
    }
}
}
void __assert_func (const char *, int, const char *, const char *){
    while(true) {
    }
}
