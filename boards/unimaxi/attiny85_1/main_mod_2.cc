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
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/usi.h"
#include "mcu/ports.h"
#include "mcu/avr/delay.h"
#include "mcu/i2cslave.h"
#include "external/tle5205.h"

#include <initializer_list>

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

//using pwmTimer = AVR::Timer8Bit<1>;
using pwm1Pin = AVR::Pin<PortB, 1>; // modifiziert
using pwm2Pin = AVR::Pin<PortB, 4>;
using pwmEPin = AVR::Pin<PortB, 3>; // modifiziert
using hbridge = TLE5205Hard<1, pwmEPin>;

constexpr TWI::Address address{0x54_B};
using Usi = AVR::Usi<0>;
using i2c = I2C::I2CSlave<Usi, address, 2>;

using isrRegistrar = IsrRegistrar<i2c::I2CSlaveHandlerOvfl, i2c::I2CSlaveHandlerStart>;

static constexpr std::hertz pwmFreq = 256 * 1000_Hz;

int main() {
    isrRegistrar::init();
    i2c::init();
    
    hbridge::init<pwmFreq>();
    
//    hbridge::direction() = TLE5205Base::Direction{false};

    for(uint8_t i = 0; i < 10; ++i) {
        hbridge::pwm(std::percent{10});
        Util::delay(300_ms);
        hbridge::pwm(std::percent{0});
        Util::delay(300_ms);
    }    
    
    {
        Scoped<EnableInterrupt<>> ei;
        while(true) {
        }
    }
}
ISR(USI_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Usi<0>::Overflow>();
}
ISR(USI_START_vect) {
    isrRegistrar::isr<AVR::ISR::Usi<0>::Start>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
    while(true) {}
}
#endif
