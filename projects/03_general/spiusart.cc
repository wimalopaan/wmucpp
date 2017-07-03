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

#include "spiusart.h"
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

using spiInput = AVR::Spi<0>;

using terminalDevice = AVR::Usart<0>;
using terminal = std::basic_ostream<terminalDevice>;

using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using ledPin = AVR::Pin<PortD, 2>;
using led = WS2812<1, ledPin>;

typedef led::color_type Color;

using systemClock = AVR::Timer8Bit<0>;
using systemTimer = AlarmTimer<systemClock>;

using sampler = PeriodicGroup<0, AVR::ISR::Timer<0>::CompareA, systemTimer>;

using isrReg = IsrRegistrar<sampler, spiInput, terminalDevice::RxHandler, terminalDevice::TxHandler>; 

template<typename Led>
class Blinker {
public:
    enum class State {Normal, Failure1, Failure2, Off, NumberOfStates};
    static void init() {
        Led::init();
    }
    static void tick() {
        static uint8_t counter = 0;
        led::set(mStateColors[(int)mState]);
        switch(mState) {
        case State::Normal:
            mState = State::Off;
            break;
        case State::Failure1:
            ++counter;
            mState = State::Failure2;
            break;
        case State::Failure2:
            if (counter < 10) {
                mState = State::Failure1;
            }
            else {
                mState = State::Normal;
                counter = 0;
            }
            break;
        case State::Off:
            mState = State::Normal;
            break;
        default:
            assert(false);
            break;
        }
    }
    static void failure() {
        mState = State::Failure1;
    }

private:
    inline static std::array<Color, (uint8_t)State::NumberOfStates> mStateColors = {
                                                                            Color{Green{128}},
                                                                            Color{Red{128}},
                                                                            Color{Red{64}, Green{0}, Blue{64}},
                                                                            Color{0},
                                                                            };

    inline static State mState = State::Off;
};

using blinker = Blinker<led>;


struct Spi0handler: public EventHandler<EventType::Spi0> {
    static bool process(std::byte v) {
        Util::put<terminalDevice, true>(v);
        return true;
    }
};

struct Timerhandler: public EventHandler<EventType::Timer> {
    static bool process(std::byte) {
        blinker::tick();
        return true;
    }
};

int main()
{
    isrReg::init();
    blinker::init();
    systemTimer::init();
    terminalDevice::init<19200>();
    
    std::outl<terminal>("Spi Usart Bridge 0.9"_pgm);
    std::outl<terminal>(Config());
    
    systemTimer::create(500_ms, AlarmFlags::Periodic);
    
    spiInput::init<AVR::SpiSlave<>>();
    
    using handler = EventHandlerGroup<Spi0handler, Timerhandler>;
    
    {
        Scoped<EnableInterrupt<>> interruptEnabler;
        EventManager::run<sampler, handler>([](){
            if (spiInput::leak()) {
                blinker::failure();
            }
        });
    }
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, ',', file, ',', line);
    while(true) {}
}
#endif

//ISR(TIMER1_COMPA_vect) {
//    //    isrReg::isr<AVR::ISR::Timer<1>::CompareA>();
//    //    SWUsart<0>::isr_compa();
//}
//ISR(TIMER1_COMPB_vect) {
//    //    isrReg::isr<AVR::ISR::Timer<1>::CompareB>();
//    //    SWUsart<0>::isr_compb();
//}
//ISR(TIMER1_CAPT_vect) {
//    //    isrReg::isr<AVR::ISR::Timer<1>::Capture>();
//    //    SWUsart<0>::isr_icp();
//}
ISR(SPI_STC_vect) {
    isrReg::isr<AVR::ISR::Spi<0>::Stc>();
}
ISR(TIMER0_COMPA_vect) {
    isrReg::isr<AVR::ISR::Timer<0>::CompareA>();
}
ISR(USART_RX_vect) {
    isrReg::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART_UDRE_vect){
    isrReg::isr<AVR::ISR::Usart<0>::UDREmpty>();
}
