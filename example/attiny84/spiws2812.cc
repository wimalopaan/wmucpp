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

#include "config.h"
#include "external/ws2812.h"
#include "mcu/avr/delay.h"
#include "mcu/avr/usi.h"
#include "mcu/avr/pinchange.h"
#include "hal/softspimaster.h"
#include "util/disable.h"
#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

using led = AVR::Pin<PortB, 0>;

template<typename Leds>
class Inserter final {
    static constexpr uint8_t* data = reinterpret_cast<uint8_t*>(&Leds::elementAt(0));
    Inserter() = delete;
public:
    static inline void insert(uint8_t b) {
        data[insertPosition] = b;        
        insertPosition = (insertPosition + 1) % (Leds::size * sizeof(typename Leds::item_type));
    }
    static inline void start() {
        insertPosition = 0;
    }
private:
    volatile static uint8_t insertPosition;
    
};
template<typename Leds>
volatile uint8_t Inserter<Leds>::insertPosition = 0;


using ws2812_Pin = AVR::Pin<PortB, 1>;
using leds = WS2812<64, ws2812_Pin>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

using inserter = Inserter<leds>;

using usiSS = AVR::Pin<PortA, 5>;
using usiSSSet = AVR::PinSet<usiSS>;
using usi = AVR::Usi<0, usiSS, inserter>;
using usiPinChange = AVR::PinChange<usiSSSet>;

using terminalDevice = SSpi0;
using terminal = std::basic_ostream<terminalDevice>;

struct PCHandler : public IsrBaseHandler<AVR::ISR::PcInt<0>> {
    static inline void isr() {
        if (!usiSS::read()) {
            inserter::start();
        }
        else {
            ++stopped;
        }
    }
    static volatile uint8_t stopped;
};
volatile uint8_t PCHandler::stopped = 0;

using isrRegistrar = IsrRegistrar<PCHandler, usi>;

int main() 
{
    isrRegistrar::init();
    terminalDevice::init();
    
    usiPinChange::init();
    
    Set<led>::output();
    led::high();
    
    leds::init();
    leds::off();
    
    usi::init();

    std::outl<terminal>("attiny usi ws2812 test"_pgm);
    
    {
        Scoped<EnableInterrupt<>> ei;
        while(true) {
            if (PCHandler::stopped > 0) {
                PCHandler::stopped = 0;
                leds::write();
            }
        }
    }
}

ISR(PCINT0_vect) {
    isrRegistrar::isr<AVR::ISR::PcInt<0>>();
}

ISR(USI_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Usi<0>::Overflow>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, ',', file, ',', line);
    while(true) {}
}
#endif
