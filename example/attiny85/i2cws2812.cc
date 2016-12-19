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
#include "util/disable.h"
#include "external/ws2812.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

static constexpr uint8_t NLeds = 3;

using ws2812_Pin = AVR::Pin<PortB, 4>;
using leds = WS2812<NLeds, ws2812_Pin>;

template<typename Leds>
class LedMachine final {
public:
    static constexpr uint8_t size = Leds::size;
    typedef Leds led_type;
    
    static volatile uint8_t& cell(uint8_t index) {
        needUpdate = true;
        if (index == 0) {
            return mColor.r;
        }
        else if (index == 1) {
            return mColor.g;
        }
        return mColor.b;
    }
    static void process() {
        if (needUpdate) {
            cRGB c{mColor.r, mColor.g, mColor.b};
            Leds::set(c);
            needUpdate = false;
        }
    }
private:
    volatile static bool needUpdate;
    volatile static cRGB mColor;
};
template<typename Leds>
volatile cRGB LedMachine<Leds>::mColor;
template<typename Leds>
volatile bool LedMachine<Leds>::needUpdate = false;

using led = AVR::Pin<PortB, 3>;

using virtualLED= LedMachine<leds>;

constexpr TWI::Address address{0x54};
using Usi = AVR::Usi<0>;
using i2c = I2C::I2CSlave<Usi, address, virtualLED>;

using isrRegistrar = IsrRegistrar<i2c::I2CSlaveHandlerOvfl, i2c::I2CSlaveHandlerStart>;

int main() 
{
    isrRegistrar::init();
    led::dir<AVR::Output>();
    led::high();
    
    i2c::init();
    
    leds::init();
    leds::off();
    
    while(true) {
        Scoped<EnableInterrupt> ei;
        led::toggle();
        virtualLED::process();
    }    
}
ISR(USI_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Usi<0>::Overflow>();
}
ISR(USI_START_vect) {
    isrRegistrar::isr<AVR::ISR::Usi<0>::Start>();
}

#ifndef NDEBUG
void assertFunction(const char*, const char*, const char*, unsigned int) {
    while(true) {};
}
#endif