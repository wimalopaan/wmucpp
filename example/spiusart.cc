/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

#include <stdlib.h>

#include "spiusart.h"
#include "mcu.h"
#include "mcutimer.h"
#include "usart.h"
#include "swusart.h"
#include "event.h"
#include "ports.h"
#include "softtimer.h"
#include "literals.h"
#include "isr.h"
#include "spi.h"
#include "ws2812.h"
#include "delay.h"
#include "console.h"

// 16 MHz full swing
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xf7:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m


using spiInput = AVR::Spi<0>;
using usartOutput = AVR::Usart<0>;
//using terminal = SWUsart<0>;
using terminal = AVR::Usart<0>;

using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using ledBlink = AVR::Pin<PortD, 7>;
using ledOverrun = AVR::Pin<PortD, 6>;

using systemClock = AVR::Timer8Bit<0>;
using systemTimer = Timer<systemClock>;

using sampler = PeriodicGroup<systemTimer>;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

class Spi0handler: public EventHandler<EventType::Spi0> {
public:
    static void process(const uint8_t& v) {
        Util::put<terminal, true>((char)v);
    }
};

class Timerhandler: public EventHandler<EventType::Timer> {
public:
    static void process(const uint8_t&) {
        ledBlink::toggle();
        if (spiInput::leak()) {
            ledOverrun::on();
        }
        else {
            ledOverrun::off();
        }
    }
};


int main()
{
    Scoped<EnableInterrupt> interruptEnabler;
    systemTimer::init();

    terminal::init<19200>();
    usartOutput::init<19200>();

    std::cout << "Spi Usart Bridge 0.1" << std::endl;

    std::cout << Config() << std::endl;

    systemTimer::create(1_s, TimerFlags::Periodic);

    spiInput::init<AVR::SpiSlave>();

    using handler = EventHandlerGroup<Spi0handler, Timerhandler>;

    EventManager::run<sampler, handler>();

    return 0;
}

void assertFunction(bool b, const char* function, const char* file, unsigned int line) {
    if (!b) {
        std::cout << "Assertion failed: " << function << "," << file << "," << line << std::endl;
        abort();
    }
}

ISR(TIMER1_COMPA_vect) {
    SWUsart<0>::isr_compa();
}
ISR(TIMER1_COMPB_vect) {
    SWUsart<0>::isr_compb();
}
ISR(TIMER1_CAPT_vect) {
    SWUsart<0>::isr_icp();
}

ISR(SPI_STC_vect) {
    AVR::Spi<0>::isr();
}

ISR(TIMER0_COMPA_vect) {
    ++sampler::tickCounter;
}

ISR(USART_RX_vect) {
    AVR::Usart<0>::rx_isr();
}
ISR(USART_UDRE_vect){
    AVR::Usart<0>::tx_isr();
}
