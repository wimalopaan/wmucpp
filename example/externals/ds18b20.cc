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
#include "external/ds18b20.h"
#include "hal/softspimaster.h"
#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

using terminal = SSpi0;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

using oneWirePin = AVR::Pin<PortA, 5>;
using oneWireMaster = OneWire::Master<oneWirePin, OneWire::Normal>;
using ds18b20 = DS18B20<oneWireMaster>;

std::array<OneWire::ow_rom_t, 5> dsIds;

int main()
{
    terminal::init();
    ds18b20::init();
    
    oneWireMaster::findDevices(dsIds);
    for(const auto& id : dsIds) {
        std::cout << id << std::endl;
    }

    while (true) {
        ds18b20::convert();
        Util::delay(750_ms);
        for(const auto& id : dsIds) {
            if (id) {
                ds18b20::ds18b20_rsp_t sp;
                ds18b20::readScratchpad(id, sp);
                auto t = ds18b20::temperature(sp);
                std::cout << "temperature "_pgm << id << " : "_pgm << t << std::endl;
            }
        }
    }
}

#ifndef NDEBUG

void assertFunction(bool b, const char* function, const char* file, unsigned int line) {
    if (!b) {
        std::cout << "Assertion failed: "_pgm << function << ","_pgm << file << ","_pgm << line << std::endl;
        abort();
    }
}
#endif
