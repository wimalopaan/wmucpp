/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <stdlib.h>

#include "mcu/ports.h"
#include "mcu/avr/twimaster.h"
#include "external/i2cram.h"
#include "hal/softspimaster.h"
#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

static constexpr std::hertz fI2C{100000};
static constexpr TWI::Address i2cramAddress{0x54};
using TwiMaster = TWI::Master<0>;
using i2cram = I2CRam<TwiMaster, i2cramAddress>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

using terminal = SSpi0;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

int main()
{
    terminal::init();

    std::cout << "i2cram (attiny) example"_pgm << std::endl;
    
    TwiMaster::init<fI2C>();

    std::array<TWI::Address, 5> i2cAddresses;
    TwiMaster::findDevices(i2cAddresses);
    for(const auto& d : i2cAddresses) {
        std::cout << d << std::endl;
    }
    
    uint8_t c = 0;
    while (true) {
        Util::delay(750_ms);
        i2cram::write(0, ++c);
        if (auto v = i2cram::read(8)) {
            std::cout << "Ram: "_pgm << *v << std::endl;
        }
    }
}

#ifndef NDEBUG
void assertFunction(bool b, const char* function, const char* file, unsigned int line) {
    if (!b) {
        std::cout << "Assertion failed: "_pgm << function << ","_pgm << file << ","_pgm << line << std::endl;
        while(true) {};
    }
}
#endif