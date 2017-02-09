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

#include "hal/alarmtimer.h"

#include "external/ws2812.h"

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using ledPin = AVR::Pin<PortB, 1>;
using led = WS2812<1, ledPin>;

typedef led::color_type Color;

using i2cInterruptPin = AVR::Pin<PortB, 0>;

constexpr auto interruptPulseWidth = 10_us;
using i2cInterrupt = AVR::SinglePulse<i2cInterruptPin, interruptPulseWidth>;

template<uint8_t NumberOfRegisters>
class RegisterMachine final {
public:
    static constexpr uint8_t size = NumberOfRegisters;
    static volatile uint8_t& cell(uint8_t index) {
        assert(index < mData.size);
        return mData[index];        
    }
    static void clear()  {
        for(uint8_t i = 0; i < mData.size; ++i) {
            mData[i] = 0;
        }
    }
//private:
    static volatile std::array<uint8_t, NumberOfRegisters> mData;
};
template<uint8_t NumberOfRegisters>
volatile std::array<uint8_t, NumberOfRegisters> RegisterMachine<NumberOfRegisters>::mData;

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

using virtualRAM = RegisterMachine<2>;

constexpr TWI::Address address{0x53};
using Usi = AVR::Usi<0>;
using i2c = I2C::I2CSlave<Usi, address, virtualRAM>;

using isrRegistrar = IsrRegistrar<i2c::I2CSlaveHandlerOvfl, i2c::I2CSlaveHandlerStart>;

int main() 
{
    isrRegistrar::init();
    
    led::init();
    led::off();
    
    i2cInterrupt::init();
    
    i2c::init();

    virtualRAM::clear();
    
    {
        Color red{128};
        Scoped<EnableInterrupt> ei;
        while(true) {
            static uint8_t counter = 0;
            Util::delay(100_ms);
            if (++counter % 2) {
                led::set(red);
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