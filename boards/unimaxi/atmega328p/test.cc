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

#include <stdlib.h>

#include "main.h"
#include "mcu/avr8.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/swusart.h"
#include "hal/event.h"
#include "mcu/ports.h"
#include "hal/alarmtimer.h"
#include "std/literals.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/spi.h"
#include "external/ws2812.h"
#include "mcu/avr/delay.h"
#include "console.h"

// 16 MHz full swing
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xf7:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

// 8Mhz int
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

//using spiInput = AVR::Spi<0>;
//using terminal = AVR::Usart<0>;

using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using ledPin = AVR::Pin<PortD, 4>;
using led = WS2812<2, ledPin>;

typedef led::color_type Color;

using systemClock = AVR::Timer8Bit<0>;
using systemTimer = AlarmTimer<systemClock>;

using sampler = PeriodicGroup<AVR::ISR::Timer<0>::CompareA, systemTimer>;

using isrReg = IsrRegistrar<
sampler
//, spiInput
//, terminal::RxHandler, terminal::TxHandler
>; 

//template<typename Led>
//class Blinker {
//public:
//    enum class State {Normal, Failure1, Failure2, Off, NumberOfStates};
//    static void init() {
//        Led::init();
//        Led::off();
//    }
//    static void tick() {
//        static uint8_t counter = 0;
//        led::set(mStateColors[(int)mState]);
//        switch(mState) {
//        case State::Normal:
//            mState = State::Off;
//            break;
//        case State::Failure1:
//            ++counter;
//            mState = State::Failure2;
//            break;
//        case State::Failure2:
//            if (counter < 10) {
//                mState = State::Failure1;
//            }
//            else {
//                mState = State::Normal;
//                counter = 0;
//            }
//            break;
//        case State::Off:
//            mState = State::Normal;
//            break;
//        default:
//            assert(false);
//            break;
//        }
//    }
//    static void failure() {
//        mState = State::Failure1;
//    }
//private:
//    static std::array<cRGB, (uint8_t)State::NumberOfStates> mStateColors;
//    static State mState;
//};
//template<typename Led>
//typename Blinker<Led>::State Blinker<Led>::mState = Blinker<Led>::State::Off;
//template<typename Led>
//std::array<cRGB, (uint8_t)Blinker<Led>::State::NumberOfStates> Blinker<Led>::mStateColors = {
//                                                                                            cRGB{0, 128, 0},
//                                                                                            cRGB{128, 0, 0},
//                                                                                            cRGB{64, 0, 64},
//                                                                                            cRGB{0, 0, 0},
//                                                                                            };

//using blinker = Blinker<led>;

//namespace std {
//std::basic_ostream<terminal> cout;
//std::lineTerminator<CRLF> endl;
//}

//struct Spi0handler: public EventHandler<EventType::Spi0> {
//    static void process(const uint8_t& v) {
//        Util::put<terminal, true>((char)v);
//    }
//};

struct Timerhandler: public EventHandler<EventType::Timer> {
    static bool process(std::byte) {
        static uint8_t counter = 0;
        ++counter;
        if (counter % 2) {
            led::off();
        }
        else {
            Color c{64};
            led::set(c);
        }
//        blinker::tick();
        return true;
    }
};

int main()
{
    isrReg::init();
//    blinker::init();
    systemTimer::init();
//    terminal::init<19200>();
    
    led::init();
    led::off();
    
//    std::cout << "UniMaxi SpiUsartBridge 0.9"_pgm << std::endl;
    
//    std::cout << Config() << std::endl;
    
    systemTimer::create(500_ms, AlarmFlags::Periodic);
    
//    spiInput::init<AVR::SpiSlave>();
    
    using handler = EventHandlerGroup<
//        Spi0handler, 
        Timerhandler>;
    
    {
        Scoped<EnableInterrupt> interruptEnabler;
        EventManager::run<sampler, handler>([](){
//            if (spiInput::leak()) {
//                blinker::failure();
//            }
        });
    }
}

#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView& , unsigned int ) noexcept {
//    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif

ISR(TIMER1_COMPA_vect) {
    //    isrReg::isr<AVR::ISR::Timer<1>::CompareA>();
}
ISR(TIMER1_COMPB_vect) {
    //    isrReg::isr<AVR::ISR::Timer<1>::CompareB>();
}
ISR(TIMER1_CAPT_vect) {
    //    isrReg::isr<AVR::ISR::Timer<1>::Capture>();
}
ISR(SPI_STC_vect) {
//    isrReg::isr<AVR::ISR::Spi<0>::Stc>();
}
ISR(TIMER0_COMPA_vect) {
    isrReg::isr<AVR::ISR::Timer<0>::CompareA>();
}
ISR(USART_RX_vect) {
//    isrReg::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART_UDRE_vect){
//    isrReg::isr<AVR::ISR::Usart<0>::UDREmpty>();
}
