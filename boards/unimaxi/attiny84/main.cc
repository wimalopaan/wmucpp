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

// 8 Mhz, 6k 4ms, 2,7V Brownout
//  sudo avrdude -p attiny84 -P usb -c avrisp2 -U lfuse:w:0xd2:m -U hfuse:w:0xdd:m -U efuse:w:0xff:m

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/i2cslave.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/delay.h"
#include "mcu/avr/usi.h"
#include "mcu/avr/swusart.h"
#include "hal/softppm.h"
#include "hal/alarmtimer.h"
#include "external/ws2812.h"
#include "console.h"

//#define USE_PPM
#define USE_SWUART

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

using ledPin = AVR::Pin<PortB, 1>;
using led = WS2812<2, ledPin>;
typedef led::color_type Color;

using leds2Pin = AVR::Pin<PortA, 2>;
using leds2 = WS2812<4, leds2Pin>;

using ppm3Out = AVR::Pin<PortB, 2>;
using ppm4Out = AVR::Pin<PortA, 7>;
#ifdef USE_PPM
using ppmTimer = AVR::Timer16Bit<1>;
using ppm = SoftPPM<ppmTimer, ppm3Out, ppm4Out>;
#endif
using i2cInterruptPin = AVR::Pin<PortB, 0>;

constexpr auto interruptPulseWidth = 10_us;
using i2cInterrupt = AVR::SinglePulse<i2cInterruptPin, interruptPulseWidth>;

constexpr TWI::Address address{0x53_B};
using Usi = AVR::Usi<0>;
using i2c = I2C::I2CSlave<Usi, address, 2>;

#ifdef USE_SWUART
using uart = SWUsart<0>;
using terminal = std::basic_ostream<uart>;
#endif

using isrRegistrar = IsrRegistrar<i2c::I2CSlaveHandlerOvfl, i2c::I2CSlaveHandlerStart
#ifdef USE_PPM
                                  ,ppm::OCAHandler, ppm::OCBHandler
#endif
#ifdef USE_SWUART
                                  ,uart::TransmitBitHandler
#endif
>;

int main() 
{
    isrRegistrar::init();
    
    led::init();
    led::off();
    
    leds2::init();
    leds2::off();
    
#ifdef USE_PPM
    ppm::init();
#endif
    
    i2cInterrupt::init();
    
    i2c::init();

    {
        Scoped<EnableInterrupt<>> ei;
        
#ifdef USE_SWUART
        std::outl<terminal>("attiny84"_pgm);
#endif
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
#ifdef USE_PPM
ISR(TIM1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}
ISR(TIM1_COMPB_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareB>();
}
#endif
#ifdef USE_SWUART
ISR(TIM1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}
#endif

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