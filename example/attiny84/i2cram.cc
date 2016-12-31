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

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usi.h"
#include "mcu/i2cslave.h"
#include "hal/softspimaster.h"
#include "util/disable.h"
#include "external/ws2812.h"

#include "console.h"

template<uint8_t NumberOfRegisters>
class RegisterMachine final {
public:
    static constexpr uint8_t size = NumberOfRegisters;
    static volatile uint8_t& cell(uint8_t index) {
        assert(index < mData.size);
        return mData[index];        
    }
    static volatile std::array<uint8_t, NumberOfRegisters> mData;
};
template<uint8_t NumberOfRegisters>
volatile std::array<uint8_t, NumberOfRegisters> RegisterMachine<NumberOfRegisters>::mData;

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

using led = AVR::Pin<PortB, 0>;
using ws2812pin = AVR::Pin<PortB, 1>;
using leds = WS2812<60, ws2812pin>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

using virtualRAM = RegisterMachine<2>;

constexpr TWI::Address address{0x53};
using Usi = AVR::Usi<0>;
using i2c = I2C::I2CSlave<Usi, address, virtualRAM>;

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
            cRGB color = {virtualRAM::mData[0] % 5, 1, 1};
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
void assertFunction(const char*, const char* function, const char* file, unsigned int line) {
    std::cout << "Assertion failed: "_pgm << function << ","_pgm << file << ","_pgm << line << std::endl;
    while(true) {};
}
#endif