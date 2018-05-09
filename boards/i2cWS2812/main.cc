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

// 16 MHz intern
// sudo avrdude -p attiny85 -P usb -c avrisp2 -U lfuse:w:0xd1:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

// sudo avrdude -p attiny85 -P usb -c avrisp2 -U flash:w:main.elf

// Register-Format:
// Byte 0     :  ControlByte
// Byte 1 - N :  LedControl
//
// ControlByte:
// ledControl:  [2Bit Mode | 6Bit Color] : Mode = Off, On, Blink, InversBlink

//#define NDEBUG

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usi.h"
#include "mcu/i2cslave.h"
#include "hal/alarmtimer.h"
#include "util/disable.h"
#include "external/ws2812.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

using systemTimer = AVR::Timer8Bit<0>;
using alarmTimer  = AlarmTimer<systemTimer, UseEvents<false>>;

static constexpr uint8_t NLeds = 16;

using ws2812_Pin = AVR::Pin<PortB, 3>;
using leds = WS2812<NLeds, ws2812_Pin>;

using led = AVR::Pin<PortB, 4>; // Pin Nr. 3

typedef leds::color_type Color;


constexpr TWI::Address address{std::byte{0x54}};
using usi = AVR::Usi<0>;
using i2c = I2C::I2CSlave<usi, address, 8>;


template<typename Leds, typename Dev>
class LedMachine final {
    enum class ColorProgram : uint8_t {Off, 
                                       Red, Green, Blue, Magenta, Yellow};
public:
    static constexpr uint8_t size = Leds::size;
    typedef Leds led_type;
    
    static void process() {
        Scoped<DisbaleInterrupt<>> di;
        if (Dev::hasChanged()) {
            
        }
    }
private:
    inline volatile static Color mColor;
};

using virtualLED = LedMachine<leds, i2c>;

using isrRegistrar = IsrRegistrar<i2c::I2CSlaveHandlerOvfl, i2c::I2CSlaveHandlerStart>;

const auto secondsTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);

static constexpr auto f = 100_Hz;

int main() {
    isrRegistrar::init();
    systemTimer::setup<f>(AVR::TimerMode::CTCNoInt);
    
    led::dir<AVR::Output>();
    led::high();
    
    i2c::init();
    
    leds::init();
    leds::off();
    leds::set(Red{10});
    
    Scoped<EnableInterrupt<>> ei;
    while(true) {
        virtualLED::process();
        // todo: ocf0a hängt ab von Timer Nr -> schlecht
        systemTimer::periodic<systemTimer::flags_type::ocf0a>([](){
            alarmTimer::periodic([](uint7_t timer){
                if (timer = *secondsTimer) {
                    led::toggle();
                }                
            });
        });
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
    while(true) {
        led::toggle();
    }
}
#endif
