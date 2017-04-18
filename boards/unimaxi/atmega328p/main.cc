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
#include "mcu/ports.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/swusart.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/spi.h"
#include "mcu/avr/delay.h"
#include "mcu/avr/twislave.h"
#include "hal/event.h"
#include "hal/alarmtimer.h"
#include "hal/softpwm.h"
#include "hal/bufferedstream.h"
#include "hal/constantrate.h"
#include "hal/button.h"
#include "hal/rotaryencoder.h"
#include "units/percent.h"
#include "std/literals.h"
#include "external/ws2812.h"
#include "external/lcd.h"
#include "console.h"

// 16 MHz full swing
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xf7:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

// 8Mhz int
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

using spiInput = AVR::Spi<0>;
using terminal = AVR::Usart<0>;

using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using ledPin = AVR::Pin<PortD, 4>;
using led = WS2812<1, ledPin>;
typedef led::color_type Color;

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using fetPin = AVR::Pin<PortB, 1>;
using lcdPwm = SoftPWM<fetPin>;

using buttonPin = AVR::Pin<PortB, 0>;
using button0 = Button<0, buttonPin>;
using buttonController = ButtonController<button0>;

using rotaryPin1 = AVR::Pin<PortD, 2>;
using rotaryPin2 = AVR::Pin<PortD, 3>;
using rotaryEncoder = RotaryEncoder<rotaryPin1, rotaryPin2>;

using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using LcdDB4 = AVR::Pin<PortC, 0>;
using LcdDB5 = AVR::Pin<PortC, 1>;
using LcdDB6 = AVR::Pin<PortC, 2>;
using LcdDB7 = AVR::Pin<PortC, 3>;

using LcdRS = AVR::Pin<PortD, 6>;
using LcdRW = AVR::Pin<PortD, 5>;
using LcdE  = AVR::Pin<PortD, 7>;

using LcdData = AVR::PinSet<LcdDB4, LcdDB5, LcdDB6, LcdDB7>;

using lcd = LCD::HD44780<LcdData, LcdRS, LcdRW, LcdE, LCD::Lcd2x16>;
using lcdStream = BufferedStream<lcd, 64>;

using systemClock = AVR::Timer8Bit<0>;
using systemTimer = AlarmTimer<systemClock>;

using systemConstantRate = ConstantRateAdapter<void, AVR::ISR::Timer<0>::CompareA, systemTimer, lcdPwm, 
                                                buttonController, rotaryEncoder>;

template<typename Led>
class Blinker {
public:
    enum class State {Normal, Failure0_1, Failure0_2, Failure1_1, Failure1_2, Off, NumberOfStates};
    static constexpr uint8_t failureBlinkCount = 3;
    static void init() {
        Led::init();
        Led::off();
    }
    static void tick() {
        static uint8_t counter = 0;
        led::set(mStateColors[(int)mState]);
        switch(mState) {
        case State::Normal:
            mState = State::Off;
            break;
        case State::Failure0_1:
            ++counter;
            mState = State::Failure0_2;
            break;
        case State::Failure0_2:
            if (counter < failureBlinkCount) {
                mState = State::Failure0_1;
            }
            else {
                mState = State::Normal;
                counter = 0;
            }
            break;
        case State::Failure1_1:
            ++counter;
            mState = State::Failure1_2;
            break;
        case State::Failure1_2:
            if (counter < failureBlinkCount) {
                mState = State::Failure1_1;
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
    static void failure0() {
        mState = State::Failure0_1;
    }
    static void failure1() {
        mState = State::Failure1_1;
    }

private:
    static std::array<Color, (uint8_t)State::NumberOfStates> mStateColors;
    static State mState;
};
template<typename Led>
typename Blinker<Led>::State Blinker<Led>::mState = Blinker<Led>::State::Off;
template<typename Led>
std::array<Color, (uint8_t)Blinker<Led>::State::NumberOfStates> Blinker<Led>::mStateColors = {
                                                                                            Color{Green{128}},
                                                                                            Color{Red{128}},
                                                                                            Color{0},
                                                                                            Color{Blue{128}},
                                                                                            Color{0},
                                                                                            Color{0},
                                                                                            };

using blinker = Blinker<led>;

constexpr TWI::Address address{0x59};
using i2c = TWI::Slave<0, address, lcd::param_type::rows * lcd::param_type::cols>;

using isrReg = IsrRegistrar<systemConstantRate, spiInput, terminal::RxHandler, terminal::TxHandler, i2c>; 

namespace std {
std::basic_ostream<terminal> cout;
std::lineTerminator<CRLF> endl;
}

std::basic_ostream<lcdStream> lcdcout;
std::lineTerminator<std::LF> lcdendl;

struct Spi0handler: public EventHandler<EventType::Spi0> {
    static bool process(const uint8_t& v) {
        Util::put<terminal, true>((char)v);
        return true;
    }
};

struct Timerhandler: public EventHandler<EventType::Timer> {
    static bool process(const uint8_t&) {
        using namespace std::literals::quantity;
        
        static uint8_t counter = 0;
        ++counter;
        
        std::percent v = std::scale(counter % 10, 0, 9);
        
        lcdPwm::pwm(v, 0);
        blinker::tick();

        lcd::home();
        lcdcout << "R: " << rotaryEncoder::value();
        
        return true;
    }
};

int main()
{
    isrReg::init();
    blinker::init();
    systemTimer::init();
    terminal::init<19200>();
    lcdPwm::init();    
    lcd::init();
    i2c::init();
    buttonController::init();
    rotaryEncoder::init();
    
    std::cout << "UniMaxi (HW 0.2) m328 (Spi Uart Lcd) 0.91"_pgm << std::endl;
    lcdcout << "UniMaxi (HW 0.2) m328 (Spi Uart Lcd) 0.91"_pgm << lcdendl;
    
    std::cout << Config() << std::endl;
    
    systemTimer::create(500_ms, AlarmFlags::Periodic);
    
    spiInput::init<AVR::SpiSlave<>>();
    
    using handler = EventHandlerGroup<Spi0handler, Timerhandler>;
    
    std::fill(i2c::registers().begin(), i2c::registers().end(), ' ');
    {
        Scoped<EnableInterrupt> interruptEnabler;
        EventManager::run2<handler>([](){
            systemConstantRate::periodic();
            lcdPwm::freeRun();
            lcdStream::periodic();
            if (spiInput::leak()) {
                blinker::failure0();
            }
            if (i2c::isChanged()) {
                i2c::isChanged() = false;
                lcd::put(i2c::registers());
            }
            if (EventManager::unprocessedEvent()) {
                EventManager::unprocessedEvent() = false;
                blinker::failure1();
            }
        });
    }
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
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
ISR(TWI_vect) {
    isrReg::isr<AVR::ISR::Twi<0>>();
}
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
