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

//#define SERIAL_DEBUG

//#define USE_BUTTON // disables SWD, use reset button then

#define NDEBUG // do not change: dev option
 
#define SW_VERSION 1

#include <cstdint>
#include <array>
#include <algorithm>

#include "stdapp/scheduler.h"
#include "devices.h"
#include "gfsm.h"
#include "storage.h"

struct DevsConfig;
using devs = Devices<WeAct, DevsConfig>;

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
    gfsm::updateFromEeprom();

    app::main([]{
        NVIC_EnableIRQ(USART1_IRQn);
        NVIC_EnableIRQ(USART2_IRQn);
    });
}

extern "C" {
void USART1_IRQHandler(){
    using relay = devs::relay;
    static_assert(relay::uart::number == 1);
    relay::Isr::onTransferComplete([]{});
    relay::Isr::onIdle([]{});
}
void USART2_IRQHandler(){
    using crsf = devs::crsf;
    static_assert(crsf::number == 2);
    crsf::Isr::onTransferComplete([]{});
    crsf::Isr::onIdle([]{});
}
}
void __assert_func (const char *, int, const char *, const char *){
    while(true) {
    }
}
