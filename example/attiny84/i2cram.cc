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

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usi.h"
#include "mcu/i2cslave.h"
#include "hal/softspimaster.h"
#include "util/disable.h"
#include "external/ws2812.h"

#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

using led = AVR::Pin<PortB, 0>;
using ws2812pin = AVR::Pin<PortB, 1>;
using leds = WS2812<60, ws2812pin>;

typedef leds::color_type Color;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

constexpr TWI::Address address{0x53};
using Usi = AVR::Usi<0>;
using i2c = I2C::I2CSlave<Usi, address, 2>;

using terminal = SSpi0;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

using isrRegistrar = IsrRegistrar<i2c::I2CSlaveHandlerOvfl, i2c::I2CSlaveHandlerStart>;

int main() 
{
    isrRegistrar::init();
    terminal::init();
    
    led::dir<AVR::Output>();
    led::high();

    leds::init();
    leds::off();
    
    i2c::init();

    std::cout << "attiny i2c ram slave test"_pgm << std::endl;

    {
        Scoped<EnableInterrupt> ei;
        while(true) {
            led::toggle();
            Color color = {Red{(uint8_t)(i2c::registers()[0] % 10)}, Green{0}, Blue{1}};
            leds::set(color);
            Util::delay(100_ms);
        }    
    }    
}
ISR(USI_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Usi<0>::Overflow>();
}
ISR(USI_STR_vect) {
    isrRegistrar::isr<AVR::ISR::Usi<0>::Start>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) {
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif