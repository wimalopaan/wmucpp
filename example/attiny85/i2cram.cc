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

template<uint8_t NumberOfRegisters>
class RegisterMachine final {
public:
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

using led = AVR::Pin<PortB, 3>;

using virtualRAM = RegisterMachine<16>;
//using virtualRAM = I2C::RamRegisterMachine<16>;

constexpr TWI::Address address{0x54};
using Usi = AVR::Usi<0>;
using i2c = I2C::I2CSlave<Usi, address, virtualRAM>;

using isrRegistrar = IsrRegistrar<i2c::I2CSlaveHandlerOvfl, i2c::I2CSlaveHandlerStart>;

int main() 
{
    isrRegistrar::init();
    led::dir<AVR::Output>();
    led::high();
    
    i2c::init();

    while(true) {
        Scoped<EnableInterrupt> ei;
        led::toggle();
        virtualRAM::process();
    }    
}
ISR(USI_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Usi<0>::Overflow>();
}
ISR(USI_START_vect) {
    isrRegistrar::isr<AVR::ISR::Usi<0>::Start>();
}

#ifndef NDEBUG
constexpr void assertFunction(bool b, const char* function, const char* file, unsigned int line) {
   if (!b) {
    }
}
#endif