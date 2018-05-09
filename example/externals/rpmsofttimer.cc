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

#include "mcu/ports.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/pinchange.h"
#include "hal/softspimaster.h"
#include "hal/softtimer.h"
#include "util/disable.h"
#include "external/rpm.h"
#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using reflex = AVR::Pin<PortB, 0>;
using reflexSet = AVR::PinSet<reflex>;
using reflexPinChange = AVR::PinChange<reflexSet>;

using led = AVR::Pin<PortB, 4>;

using mcuTimer = AVR::Timer8Bit<0>;
using rpmTimer = SoftTimer<mcuTimer, uint16_t>;

constexpr std::RPM MaxRpm{15000};
constexpr std::RPM MinRpm{100};
using rpm = RpmFromInterruptSource<reflexPinChange, rpmTimer, MaxRpm, MinRpm>;

using isrRegistrar = IsrRegistrar<rpm, rpmTimer>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

using terminal = SSpi0;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

int main() {
    isrRegistrar::init();
    rpm::init();
    terminal::init();
    
    led::template dir<AVR::Output>();
    led::off();
    
    {
        Scoped<EnableInterrupt<>> ei;        
        std::cout << "RPM with PinChange example"_pgm << std::endl;
        std::cout << "timer f: "_pgm << rpmTimer::frequency() << std::endl;
        std::cout << "timer p: "_pgm << rpmTimer::prescaler() << std::endl;
        
        while(true) {
            Util::delay(1000_ms);
            std::cout << "rpm: "_pgm << rpm::rpm() << std::endl;
            rpm::reset();
        }
    }
}

ISR(PCINT1_vect) {
    led::toggle();
    isrRegistrar::isr<AVR::ISR::PcInt<1>>();
}
ISR(TIMER0_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::Overflow>();
}

#ifndef NDEBUG
void assertFunction([[maybe_unused]] const PgmStringView& expr, [[maybe_unused]] const PgmStringView& file, [[maybe_unused]] unsigned int line) noexcept {
//    std::outl<terminal>("Assertion failed: "_pgm, expr, ',', file, ',', line);
    while(true) {}
}
#endif
