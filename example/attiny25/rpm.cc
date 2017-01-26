/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include "mcu/avr/isr.h"
#include "mcu/avr/pinchange.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/i2cslave.h"
#include "hal/softtimer.h"
#include "util/disable.h"
#include "external/rpm.h"

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using led = AVR::Pin<PortB, 3>;

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

using RpmMachine = RegisterMachine<4>;

constexpr TWI::Address address{0x55};
using Usi = AVR::Usi<0>;
using i2c = I2C::I2CSlave<Usi, address, RpmMachine>;

using reflex = AVR::Pin<PortB, 4>;
using reflexSet = AVR::PinSet<reflex>;
using reflexPinChange = AVR::PinChange<reflexSet>;

using mcuTimer = AVR::Timer8Bit<0>;
using rpmTimer = SoftTimer<mcuTimer, uint16_t>;

using rpm = RpmFromInterruptSource<reflexPinChange, rpmTimer>;

using isrRegistrar = IsrRegistrar<i2c::I2CSlaveHandlerOvfl, i2c::I2CSlaveHandlerStart, rpmTimer, rpm>;

int main() 
{
    led::dir<AVR::Output>();
    led::high();
    
    isrRegistrar::init();
    i2c::init();
    
    rpm::init();

    {
        Scoped<EnableInterrupt> ei;
        while(true) {
            auto pp = rpm::period();
            RpmMachine::mData[0] = pp;
            RpmMachine::mData[1] = pp >> 8;
            
            auto rr = rpm::rpm();
            RpmMachine::mData[2] = rr.mValue;
            RpmMachine::mData[3] = rr.mValue >> 8;
        }    
    }
}

ISR(USI_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Usi<0>::Overflow>();
}

ISR(USI_START_vect) {
    isrRegistrar::isr<AVR::ISR::Usi<0>::Start>();
}
ISR(PCINT0_vect) {
    led::toggle();
    isrRegistrar::isr<AVR::ISR::PcInt<0>>();
}
ISR(TIMER0_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::Overflow>();
}
//ISR(TIMER1_OVF_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<1>::Overflow>();
//}

#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
//    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif
