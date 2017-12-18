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

#include <stdlib.h>

#include "main.h"
#include "mcu/avr8.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/ports.h"
#include "mcu/avr/spi.h"

// 16 MHz full swing
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xf7:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

// 8Mhz int
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;

using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using lcdPwmPin = AVR::Pin<PortB, 1>;

using systemClock = AVR::Timer8Bit<0>;

using spi = AVR::Spi<0, AVR::SpiSlave<MCU::UseInterrupts<false>>>;

static constexpr auto systemFrequency = 100_Hz;

int main() {
    spi::init();
    
    lcdPwmPin::dir<AVR::Output>();
    lcdPwmPin::off();
    
    while(true) {
        spi::whenReady([](std::byte){
            lcdPwmPin::toggle();
        });
    }
}

#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView& , unsigned int ) noexcept {
    //    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif
