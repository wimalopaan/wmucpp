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

// sudo avrdude -p attiny84 -P usb -c avrisp2 -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usi.h"
#include "mcu/i2cslave.h"
#include "hal/softspimaster.h"
#include "console.h"

template<uint8_t NumberOfRegisters>
class RegisterMachine final {
public:
    static constexpr uint8_t size = NumberOfRegisters;
    static uint8_t& cell(uint8_t index) {
        assert(index < mData.size);
        return mData[index];        
    }
    static void process() {
        for(uint8_t i = 0; i < mData.size / 2; ++i) {
            mData[i + mData.size / 2] = mData[i] + 1;
        }
    }
private:
    static std::array<uint8_t, NumberOfRegisters> mData;
};
template<uint8_t NumberOfRegisters>
std::array<uint8_t, NumberOfRegisters> RegisterMachine<NumberOfRegisters>::mData;

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

using led = AVR::Pin<PortB, 0>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

using virtualRAM = RegisterMachine<16>;
//using virtualRAM = I2C::RamRegisterMachine<16>;

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
    terminal::init();
    
    led::dir<AVR::Output>();
    led::high();
    
    i2c::init();

    std::cout << "attiny i2c slave test"_pgm << std::endl;
    
    while(true) {
        led::toggle();
        virtualRAM::process();
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