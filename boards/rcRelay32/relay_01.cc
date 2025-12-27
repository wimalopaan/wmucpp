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

//#define USE_WEACT
#define USE_WMG0B1
// #define USE_NUCLEO_431 // dev board

// #define SERIAL_DEBUG // only for WMG0B1

//#define ALTERNATE_PINS // WmG0B1 only

#define NDEBUG // do not change: dev option

#if defined(USE_WEACT)
# define HW_VERSION 1
#elif defined(USE_WMG0B1)
# define HW_VERSION 2
// # define USE_TP
#elif defined(USE_NUCLEO_431)
# define HW_VERSION 3
#else
# warning "wrong board definition"
#endif
#define SW_VERSION 10

#include <cstdint>
#include <array>
#include <algorithm>

#include "stdapp/scheduler.h"
#include "devices.h"
#include "gfsm.h"
#include "storage.h"

struct DevsConfig;
#ifdef USE_WEACT
using devs = Devices<WeAct, DevsConfig>;
#endif
#ifdef USE_WMG0B1
using devs = Devices<Wmg0b1, DevsConfig>;
#endif
#ifdef USE_NUCLEO_431
using devs = Devices<Nucleo, DevsConfig>;
#endif

using gfsm = GFSM<devs>;

struct AppConfig;
using app = Scheduler<AppConfig>;

struct DevsConfig {
    using storage = Storage<devs>;
	using gfsm = ::gfsm;
};
struct AppConfig {
	using fsm = gfsm;
	using timer = devs::systemTimer;
};
int main() {
    DevsConfig::storage::init();

    app::main([]{
        NVIC_EnableIRQ(USART1_IRQn);
#if defined(USE_WEACT) || defined(USE_NUCLEO_431)
        NVIC_EnableIRQ(USART2_IRQn);
#elif defined(USE_WMG0B1)
#ifdef ALTERNATE_PINS
        NVIC_EnableIRQ(USART3_4_5_6_LPUART1_IRQn);
#else
        NVIC_EnableIRQ(USART2_LPUART2_IRQn);
#endif
#else
# warning "wrong board definition"
#endif
    });
}

extern "C" {
#if defined(USE_WEACT) || defined(USE_NUCLEO_431)
void USART1_IRQHandler(){
    using relay = devs::relay;
    static_assert(relay::uart::number == 1);
    relay::Isr::onTransferComplete([]{});
    relay::Isr::onIdle([]{});
}
#elif defined(USE_WMG0B1)
void USART1_IRQHandler(){
    using crsf = devs::crsf;
    static_assert(crsf::number == 1);
    crsf::Isr::onTransferComplete([]{});
    crsf::Isr::onIdle([]{});
}
#else
# warning "wrong board definition"
#endif
#if defined(USE_WEACT) || defined(USE_NUCLEO_431)
void USART2_IRQHandler(){
    using crsf = devs::crsf;
    static_assert(crsf::number == 2);
    crsf::Isr::onTransferComplete([]{});
    crsf::Isr::onIdle([]{});
}
#elif defined(USE_WMG0B1)
# ifdef ALTERNATE_PINS
void USART3_4_5_6_LPUART1_IRQHandler() {
    using relay = devs::relay;
    static_assert(relay::uart::number == 6);
    relay::Isr::onTransferComplete([]{});
    relay::Isr::onIdle([]{});
}
# else
void USART2_LPUART2_IRQHandler() {
    using relay = devs::relay;
    static_assert(relay::uart::number == 2);
    relay::Isr::onTransferComplete([]{});
    relay::Isr::onIdle([]{});
}
# endif
#else
# warning "wrong board definition"
#endif
}
void __assert_func (const char *, int, const char *, const char *){
    while(true) {
    }
}
