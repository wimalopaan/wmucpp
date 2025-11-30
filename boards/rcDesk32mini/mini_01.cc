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

// use one(!) of the following protocol selection
// #define USE_HWEXT // otherwise SBUS-output is used
// #define USE_SBUS
#define USE_CRSF
// #define USE_INVERT_SERIAL

//#define USE_BUTTON // disables SWD, use reset button then

#define NDEBUG // do not change: dev option
 
#define SW_VERSION 2

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
    app::main([]{
        NVIC_EnableIRQ(USART1_IRQn);
    });
}

extern "C" {
void USART1_IRQHandler(){
#if defined(USE_HWEXT)
	using hwext = devs::hwext;
	static_assert(hwext::uart::number == 1);
	hwext::Isr::onTransferComplete([]{});
	hwext::Isr::onIdle([]{});
#elif defined(USE_SBUS)
	using sbus = devs::sbus;
	static_assert(sbus::uart::number == 1);

	using modcom = devs::modcom;
	static_assert(modcom::number == 1);
	modcom::Isr::onTransferComplete([]{});
	modcom::Isr::onIdle([]{});
#elif defined(USE_CRSF)
	using crsf = devs::crsf;
	static_assert(crsf::uart::number == 1);
	crsf::Isr::onTransferComplete([]{});
	crsf::Isr::onIdle([]{});
#else
# warning "wrong protocol selection"
#endif
}
}
void __assert_func (const char *, int, const char *, const char *){
    while(true) {
    }
}
