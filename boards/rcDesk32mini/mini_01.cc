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

// reads 8 analog inputs (A0...A3, A5...A8) (A4 of the WeAct board is used for the LED)
// reads 4 digital inputs (B6, B7, C14, C15)
// produces a 16-channel RC data stream (CRSF, SBUS, HW-Extension) (see below)
// injects the digitals in the RC data stream (see below)
// if using CRSF, the digitals are also transported as CRSF-switch-protocol extension

// for SFrog-mode:
// using bootPress (pressing (d0 & d3) or (d1 & d2)) one can reach calibration mode at boot time
// first stage: sampling mid position
// led fast flash 1x
// then press d0 or d1
// second stage: move all analogs to full extend
// led fast flash 2x
// then press d0 or d1
// run-stage is reached

// use one(!) of the following protocol selection exclusively
// #define USE_HWEXT // otherwise SBUS-output is used
#define USE_SBUS // according SBUS inversion see below
// #define USE_CRSF
// #define USE_SUMDV1
// #define USE_SUMDV3

#define USE_SFROG // make A3, A8 outputs for Stick-LEDs

#define INJECT_DIGITAL_START 8 // first channel to inject digitals into CRSF / SBUS (counting from 0)
#define SWITCH_ADDRESS 0 // only valid for CSRF (using switch protocol extension)

// #define SBUS_SERIAL_INVERT // SBUS is an inverted serial protocol, but the TX16s can read only uninverted serial signals

// use broadcast for switch command
#define CRSF_SWITCH_COMMAND_ADDRESS RC::Protokoll::Crsf::V4::Address::Broadcast
// if the above is not working for some weird reason (older ELRS?), use the following
// #define CRSF_SWITCH_COMMAND_ADDRESS RC::Protokoll::Crsf::V4::Address::Controller

// #define SERIAL_DEBUG // disables PA2 analog input

// #define USE_BUTTON // disables SWD (use with care and know what you are doing), use reset button then

#define NDEBUG // do not change: dev option
 
#define SW_VERSION 6

#include <cstdint>
#include <array>
#include <algorithm>

#include "stdapp/scheduler.h"

#include "devices.h"
#include "gfsm.h"
#include "storage.h"

struct DevsConfig;
#ifdef USE_SFROG
using devs = Devices<WeAct_SFrog, DevsConfig>;
#else
using devs = Devices<WeAct, DevsConfig>;
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
#elif defined(USE_SUMDV3)
    using sumdv3 = devs::sumdv3;
    static_assert(sumdv3::uart::number == 1);
#elif defined(USE_SUMDV1)
    using sumdv1 = devs::sumdv1;
    static_assert(sumdv1::uart::number == 1);
#else
# warning "wrong protocol selection"
#endif
}
}
void __assert_func (const char *, int, const char *, const char *){
    while(true) {
    }
}
