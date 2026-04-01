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

#define NDEBUG // do not change: dev option

#define HW_VERSION 1
#define SW_VERSION 1

#define SERIAL_DEBUG

#include "devices.h"
#include "storage.h"
#include "gfsm.h"

#include "stdapp/scheduler.h"

struct DevsConfig;
using devs = Devices<Foc01, DevsConfig>;
using gfsm = GFSM<devs>;

struct AppConfig;
using app = Scheduler<AppConfig>;

struct DevsConfig {
    using storage = Storage;
};
struct AppConfig {
    using fsm = gfsm;
    using timer = devs::systemTimer;
};

int main() {
    Storage::init();
    app::main([]{
		gfsm::update(true);
        NVIC_EnableIRQ(USART1_IRQn);
		NVIC_EnableIRQ(ADC1_2_IRQn);
    });
}

extern "C" {

void ADC1_2_IRQHandler() {
	using adc1 = devs::adc1;
	adc1::whenSequenceComplete([]{
		devs::tp::set();
		devs::tp::reset();
	});
	using adc2 = devs::adc2;
	adc2::whenSequenceComplete([]{
		devs::tp::set();
		devs::tp::reset();
	});
}
void USART1_IRQHandler(){
    using crsf = devs::crsf;
    static_assert(crsf::number == 1);
    crsf::Isr::onTransferComplete([]{});
    crsf::Isr::onIdle([]{});
}
void HardFault_Handler() {
    while(true) {
#ifdef USE_TP
        devs::tp::set();
        devs::tp::reset();
#endif
    }
}
}

void __assert_func (const char *, int, const char *, const char *){
    while(true) {
    }
}
