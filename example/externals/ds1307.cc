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

#include <stdlib.h>

#include "mcu/ports.h"
#include "mcu/avr/twimaster.h"
#include "external/ds1307.h"
#include "hal/softspimaster.h"
#include "console.h"

static constexpr bool useTerminal = true;

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using TwiMaster = TWI::Master<0>;
using ds1307 = DS1307<TwiMaster>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

using terminalDevice = std::conditional<useTerminal, SSpi0, void>::type;
using terminal = std::basic_ostream<terminalDevice>;

namespace std {
    std::basic_ostream<terminalDevice> cout;
    std::lineTerminator<CRLF> endl;
}

int main()
{
    terminalDevice::init();

    std::cout << "DS1307 example"_pgm << std::endl;
    
    TwiMaster::init<ds1307::fSCL>();

    std::array<TWI::Address, 5> i2cAddresses;
    TwiMaster::findDevices(i2cAddresses);
    for(const auto& d : i2cAddresses) {
        std::cout << d << std::endl;
    }
    
    ds1307::init();
    ds1307::halt<false>();
    
    if (!ds1307::squareWave<false>()) {
        std::cout << "Error"_pgm << std::endl;
    }
    
    if (auto v = ds1307::readControlRegister()) {
        std::cout << "Control Register: "_pgm << *v << std::endl;
    }
    
    uint8_t c = 0;
    while (true) {
        Util::delay(750_ms);
        ds1307::writeToRam(0, ++c);
        if (auto v = ds1307::readFromRam(0)) {
            std::cout << "Ram: "_pgm << *v << std::endl;
        }
        
        if (ds1307::readTimeInfo()) {
            std::cout << "TI[0]: "_pgm << ds1307::timeInfo()[0] << std::endl;
        }
    }
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr __attribute__((unused)), const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, ',', file, ',', line);
    while(true) {}
}
#endif
