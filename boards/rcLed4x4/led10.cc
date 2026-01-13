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

// #define USE_MORSE
// #define USE_EEPROM_TEST // switches telemetry default on (instead off)
#define SERIAL_DEBUG

#define HW_LED2

#define NDEBUG // do not change: dev option

#define HW_VERSION 2
#define SW_VERSION 5

#include <cstdint>
#include <array>

#include "devices.h"
#include "storage.h"
#include "gfsm.h"

struct DevsConfig;
using devs = Devices<Led10, DevsConfig, Mcu::Stm::Stm32G0B1>;
using gfsm = GFSM<devs>;

struct DevsConfig {
	using storage = Storage;
};

int main() {
	Storage::init();
	gfsm::init();
	gfsm::update(true);
	
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	__enable_irq();
	
	while(true) {
		gfsm::periodic();
		devs::systemTimer::periodic([]{
			gfsm::ratePeriodic();
		});
	}
}

extern "C" {

void USART1_IRQHandler(){
	using crsf = devs::crsf;
	static_assert(crsf::number == 1);
	crsf::Isr::onTransferComplete([]{});
	crsf::Isr::onIdle([]{});
}
void DMA1_Channel2_3_IRQHandler() {
	using pca9745 = devs::pca9745;
	using spi = pca9745::spi;
	using dmaChW = spi::dmaChW;
	static_assert(dmaChW::number == 3);
	pca9745::Isr::onTransferComplete([]{});
}
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
