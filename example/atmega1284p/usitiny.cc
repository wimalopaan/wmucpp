/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "usitiny.h"

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "hal/softspimaster.h"
#include "util/disable.h"
#include "std/array.h"
#include "std/algorithm.h"
#include "external/ws2812.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, void, true>;

using UsiSelect = AVR::Pin<PortA, 2>;

template<typename Device>
class ByteWriter final {
    ByteWriter() = delete;
public:
    typedef Device device_type;

    template<typename C>
    static void put(const C& container) {
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&container[0]);   
        const uint8_t* end = reinterpret_cast<const uint8_t*>(&container[container.size - 1] + 1);   
        for(; ptr < end; ++ptr) {
            Device::put(*ptr);
        }
    }
 
private:
};

std::array<cRGB, 64> leds;

using byteWriter = ByteWriter<SSpi0>;

int main()
{
    UsiSelect::dir<AVR::Output>();
    UsiSelect::high();
    
    SSpi0::init();
    
    uint8_t counter = 0;
    while(true) {
        Util::delay(1000_ms);
        {
            AVR::ScopedPin<UsiSelect, AVR::ActiveLow> ss;
            Util::delay(Config::SoftSpiMaster::pulseDelay);
            uint8_t v = (counter++ % 2) * 10;
            std::fill(std::begin(leds), std::end(leds), cRGB{v, v, v});
            byteWriter::put(leds);
            Util::delay(Config::SoftSpiMaster::pulseDelay);
        }
    }    
}
#ifndef NDEBUG
void assertFunction(const char*, const char*, const char* , unsigned int) {
    while(true) {};
}
#endif