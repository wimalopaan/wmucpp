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

// external Osc (Freq see Makefile)
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xd0:m -U hfuse:w:0xd1:m -U efuse:w:0xff:m

// internal 8 MHz
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

//#define SIMAVR

#include <stdlib.h>
#include <util/eu_dst.h>

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "external/ws2812.h"
#include "console.h"

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using iRPin          = AVR::Pin<PortB, 0>;
using d0_ss_Pin      = AVR::Pin<PortD, 6>;
using d1_data_Pin    = AVR::Pin<PortD, 5>;
using d2_clck_Pin    = AVR::Pin<PortB, 3>;
using d3_Pin         = AVR::Pin<PortD, 3>;

using displayLedsPin = d0_ss_Pin;

using rxdPin         = AVR::Pin<PortD, 0>;
using txdPin         = AVR::Pin<PortD, 1>;
using dcfPin         = AVR::Pin<PortC, 2>;
using powerSwitchPin = AVR::Pin<PortD, 2>;

using ledPin         = AVR::Pin<PortB, 7>;
using led = WS2812<1, ledPin, ColorSequenceGRB>;
using Color = led::color_type;

using display = WS2812<10 * 11 + 4, displayLedsPin, ColorSequenceGRB>;

namespace Constant {
static constexpr uint8_t brightness = 255;
static constexpr Color cOff{0};
static constexpr Color cRed{Red{brightness}};
static constexpr Color cBlue{Blue{std::min((int)std::numeric_limits<uint8_t>::max(), 2 * brightness)}};
static constexpr Color cGreen{Green{brightness / 2}};
static constexpr Color cYellow{Red{brightness}, Green{brightness}, Blue{0}};
static constexpr Color cMagenta{Red{brightness}, Green{0}, Blue{brightness}};
static constexpr Color cCyan{Red{0}, Green{brightness}, Blue{brightness}};
static constexpr Color cWhite{Red{brightness}, Green{brightness}, Blue{brightness}};
static constexpr Color cWhiteLow{Red{brightness / 10}, Green{brightness / 10}, Blue{brightness / 10}};

static constexpr auto title = "Leds Test"_pgm;
}
using terminal = AVR::Usart<0, void>;

namespace std {
std::basic_ostream<terminal> cout;
std::lineTerminator<CRLF> endl;
}

using isrRegistrar = IsrRegistrar<terminal::RxHandler, terminal::TxHandler>;

struct Usart0Handler : public EventHandler<EventType::UsartRecv0> {
    static bool process(uint8_t) {
        return true;
    }
};
struct UsartFeHandler : public EventHandler<EventType::UsartFe> {
    static bool process(uint8_t) {
        return true;
    }
};
struct UsartUpeHandler : public EventHandler<EventType::UsartUpe> {
    static bool process(uint8_t) {
        return true;
    }
};
struct UsartDorHandler : public EventHandler<EventType::UsartDor> {
    static bool process(uint8_t) {
        return true;
    }
};

using allEventHandler = EventHandlerGroup<UsartFeHandler, UsartUpeHandler, UsartDorHandler, Usart0Handler>;

int main() {   
    powerSwitchPin::dir<AVR::Output>();    
    powerSwitchPin::on();    
    
    display::init();
    
    isrRegistrar::init();
    terminal::init<19200>();
    
    {
        Scoped<EnableInterrupt<>> ei;
        std::cout << Constant::title << std::endl;
        
        while(true) {
            for(uint8_t i = 0; i < display::size; ++i) {
                display::set(i, Constant::cWhiteLow);
                Util::delay(100_ms);
            }
            display::set(Constant::cOff);
        }
    }
}

ISR(USART_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif

