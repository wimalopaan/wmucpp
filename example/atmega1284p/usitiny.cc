/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include "std/array"
#include "std/algorithm"
#include "external/ws2812.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, void, true>;

using UsiSelectPin = AVR::Pin<PortA, 2>;
using UsiSelect = AVR::ActiveLow<UsiSelectPin, AVR::Output>;

template<typename Device>
class ByteWriter final {
    ByteWriter() = delete;
public:
    typedef Device device_type;

    template<typename C>
    static void put(const C& container) {
        const std::byte* ptr = reinterpret_cast<const std::byte*>(&container[0]);   
        const std::byte* end = reinterpret_cast<const std::byte*>(&container[container.size - 1] + 1);   
        for(; ptr < end; ++ptr) {
            Device::put(*ptr);
        }
    }
};

typedef cRGB<ColorSequenceRGB> Color;

std::array<Color, 64> leds;

using byteWriter = ByteWriter<SSpi0>;

int main()
{
    UsiSelect::init();    
    SSpi0::init();
    
    uint8_t counter = 0;
    while(true) {
        Util::delay(1000_ms);
        {
            AVR::ScopedPin<UsiSelect> ss;
            Util::delay(Config::SoftSpiMaster::pulseDelay);
            uint8_t v = (counter++ % 2) * 10;
            std::fill(std::begin(leds), std::end(leds), Color{Red{v}, Green{v}, Blue{v}});
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
