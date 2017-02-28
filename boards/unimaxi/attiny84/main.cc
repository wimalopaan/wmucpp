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

// 8 Mhz, 6k 4ms, 2,7V Brownout
//  sudo avrdude -p attiny84 -P usb -c avrisp2 -U lfuse:w:0xd2:m -U hfuse:w:0xdd:m -U efuse:w:0xff:m

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/i2cslave.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/delay.h"
#include "mcu/avr/usi.h"
#include "hal/softppm.h"
#include "hal/alarmtimer.h"
#include "external/ws2812.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

using ledPin = AVR::Pin<PortB, 1>;
using led = WS2812<2, ledPin>;
typedef led::color_type Color;

using leds2Pin = AVR::Pin<PortA, 2>;
using leds2 = WS2812<4, leds2Pin>;

using ppm3Out = AVR::Pin<PortB, 2>;
using ppm4Out = AVR::Pin<PortA, 7>;
using ppmTimer = AVR::Timer16Bit<1>;
using ppm = SoftPPM<ppmTimer, ppm3Out, ppm4Out>;

using i2cInterruptPin = AVR::Pin<PortB, 0>;

constexpr auto interruptPulseWidth = 10_us;
using i2cInterrupt = AVR::SinglePulse<i2cInterruptPin, interruptPulseWidth>;

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

constexpr TWI::Address address{0x53};
using Usi = AVR::Usi<0>;
using i2c = I2C::I2CSlave<Usi, address, 2>;

using isrRegistrar = IsrRegistrar<i2c::I2CSlaveHandlerOvfl, i2c::I2CSlaveHandlerStart,
                                  ppm::OCAHandler, ppm::OCBHandler>;

int main() 
{
    isrRegistrar::init();
    
    led::init();
    led::off();
    
    leds2::init();
    leds2::off();
    
    ppm::init();
    
    i2cInterrupt::init();
    
    i2c::init();

    {
        Scoped<EnableInterrupt> ei;
        while(true) {
            static uint8_t counter = 0;
            Util::delay(100_ms);
            if (++counter % 2) {
                led::set(Color(Red{16}));
            }
            else {
                led::off();
            }
            if ((counter % 10) == 0) {
                i2cInterrupt::trigger();
            }
        }    
    }    
}
ISR(TIM1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}
ISR(TIM1_COMPB_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareB>();
}

ISR(USI_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Usi<0>::Overflow>();
}
ISR(USI_STR_vect) {
    isrRegistrar::isr<AVR::ISR::Usi<0>::Start>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
    while(true) {}
}
#endif