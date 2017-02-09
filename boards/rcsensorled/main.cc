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

// sudo avrdude -p atmega328pb -P usb -c avrisp2 -U lfuse:w:0xe0:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

#include <stdlib.h>

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/swusart.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/pinchange.h"
#include "hal/event.h"
#include "hal/alarmtimer.h"
#include "hal/constantrate.h"
#include "external/ws2812.h"
#include "external/tle5205.h"
#include "external/rpm.h"
#include "console.h"

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using PortE = AVR::Port<DefaultMcuType::PortRegister, AVR::E>;

using ledPin = AVR::Pin<PortD, 6>;
using led = WS2812<1, ledPin, ColorSequenceGRB>;
typedef led::color_type Color;

using leds1Pin = AVR::Pin<PortE, 0>;
using leds2Pin = AVR::Pin<PortB, 0>;

using Tle5205In1 = AVR::Pin<PortD, 5>;
using Tle5205In2 = AVR::Pin<PortD, 4>;
using Tle5205Error = AVR::Pin<PortD, 2>;
using tleTimer = AVR::Timer8Bit<2>; // timer 2
using hbridge = TLE5205<Tle5205In1, Tle5205In2, Tle5205Error, tleTimer>;

using reflexPin = AVR::Pin<PortC, 3>;
using reflexPinSet = AVR::PinSet<reflexPin>;
using reflexPinChange = AVR::PinChange<reflexPinSet>;

using rpmTimer = AVR::Timer16Bit<1>; // timer 1
using rpm = RpmFromInterruptSource<reflexPinChange, rpmTimer>;

using terminal = SWUsart<1>; // timer 3 (icp3)

using systemClock = AVR::Timer8Bit<0>; // timer 0
using systemTimer = AlarmTimer<systemClock>;

using systemConstantRate = ConstantRateAdapter<void, AVR::ISR::Timer<0>::CompareA, systemTimer>;


using isrRegistrar = IsrRegistrar<systemConstantRate, 
                                  hbridge::PwmOnHandler, hbridge::PwmOffHandler,
                                  terminal::ReceiveBitHandler, terminal::TransmitBitHandler, terminal::StartBitHandler,
                                  rpm>;

static constexpr std::hertz pwmFrequency = 1000_Hz;

namespace std {
std::basic_ostream<terminal> cout;
std::lineTerminator<CRLF> endl;
}

int main()
{
    isrRegistrar::init();
    hbridge::init<pwmFrequency>();
    terminal::init<2400>();
    rpm::init();
    
    leds1Pin::template dir<AVR::Output>();
    leds2Pin::template dir<AVR::Output>();
    
    led::init();
    led::off();    
    
    const Color red{Red{16}};

    uint8_t counter = 0;
    {
        Scoped<EnableInterrupt> ei;

        using namespace std::literals::quantity;
        hbridge::pwm(15_ppc);
        
        std::cout << "RC SensorLed 0.1"_pgm << std::endl;
        
        while(true) {
            std::cout << "A: "_pgm << counter << std::endl;      
            Util::delay(500_ms);
            leds2Pin::toggle();
            if ((++counter % 2) == 0) {
                led::set(red);
            }
            else {
                led::off();
            }
            hbridge::periodic();
            std::cout << "rpm: "_pgm << rpm::rpm() << std::endl;
            std::cout << "per: "_pgm << rpm::period() << std::endl;
            rpm::reset();            
        }
    }
}

ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}
//ISR(TIMER1_COMPA_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
//}
ISR(TIMER2_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<2>::CompareA>();
}
ISR(TIMER2_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<2>::Overflow>();
}
ISR(TIMER3_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<3>::CompareA>();
}
ISR(TIMER3_COMPB_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<3>::CompareB>();
}
ISR(TIMER3_CAPT_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<3>::Capture>();
}
ISR(PCINT1_vect) {
    leds1Pin::toggle();
    isrRegistrar::isr<AVR::ISR::PcInt<1>>();
}


#ifndef NDEBUG
void assertFunction(const char*, const char*, const char*, unsigned int) {
       while(true) {}
}
#endif
