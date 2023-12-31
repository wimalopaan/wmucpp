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


constexpr TWI::Address address{std::byte{0x55}};
using Usi = AVR::Usi<0>;
using i2c = I2C::I2CSlave<Usi, address, 4>;

using reflex = AVR::Pin<PortB, 4>;
using reflexSet = AVR::PinSet<reflex>;
using reflexPinChange = AVR::PinChange<reflexSet>;

using mcuTimer = AVR::Timer8Bit<0>;
using rpmTimer = SoftTimer<mcuTimer, uint16_t>;

constexpr std::RPM MaximumRpm{15000};
constexpr std::RPM MinimumRpm{100};

using rpm = RpmFromInterruptSource<reflexPinChange, rpmTimer, MaximumRpm, MinimumRpm>;

using isrRegistrar = IsrRegistrar<i2c::I2CSlaveHandlerOvfl, i2c::I2CSlaveHandlerStart, rpmTimer, rpm>;

int main() 
{
    led::dir<AVR::Output>();
    led::high();
    
    isrRegistrar::init();
    i2c::init();
    
    rpm::init();

    {
        Scoped<EnableInterrupt<>> ei;
        while(true) {
            auto pp = rpm::period();
            i2c::registers()[0] = std::byte(pp);
            i2c::registers()[1] = std::byte(pp >> 8);
            
            auto rr = rpm::rpm();
            i2c::registers()[2] = std::byte(rr.value());
            i2c::registers()[3] = std::byte(rr.value() >> 8);
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
