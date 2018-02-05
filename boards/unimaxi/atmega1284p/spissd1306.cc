/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/spi.h"
#include "hal/alarmtimer.h"
#include "hal/softspimaster.h"
#include "units/duration.h"
#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

// Timer0
using systemTimer = AVR::Timer8Bit<0>;
using alarmTimer = AlarmTimer<systemTimer, UseEvents<false>>;

// Terminal to atmega328
using SoftSPIData = AVR::Pin<PortB, 1>;
using SoftSPIClock = AVR::Pin<PortB, 0>;
using SoftSPISS = AVR::Pin<PortC, 3>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS, true>;
using terminalDevice = SSpi0;
using terminal = std::basic_ostream<terminalDevice>;

// SPI for SSD1306
using ssd1306 = AVR::Spi<0, AVR::SpiMaster<MCU::UseInterrupts<false>>>;

int main() {
    terminalDevice::init();
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 
    ssd1306::init();
    
    std::outl<terminal>("sd1306 test"_pgm);
    
    const auto periodicTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);
    while(true) {
        systemTimer::periodic<systemTimer::flags_type::ocfa>([&](){
            alarmTimer::periodic([&](uint7_t timer) {
                if (timer == *periodicTimer) {
                    std::outl<terminal>("tick"_pgm);
                }
            });
        });
    }
}
#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, ',', file, ',', line);
    while(true) {}
}
#endif
