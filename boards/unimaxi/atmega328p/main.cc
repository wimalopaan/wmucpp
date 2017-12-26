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

#define NDEBUG

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

using spiInput = AVR::Spi<0, AVR::SpiSlave<MCU::UseInterrupts<false>>>;
//using spiInput = AVR::Spi<0, AVR::SpiSlave<MCU::UseInterrupts<true>>>;

using terminalDevive = AVR::Usart<0, void, MCU::UseInterrupts<true>, UseEvents<true>, AVR::ReceiveQueueLength<0>>;
using terminal = std::basic_ostream<terminalDevive>;

using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using ledPin = AVR::Pin<PortD, 4>;
using led = WS2812<1, ledPin>;
typedef led::color_type Color;

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using fetPin = AVR::Pin<PortB, 1>;
using lcdPinSet = AVR::PinSet<fetPin>;
using lcdPwm = HAL::SoftPWM<lcdPinSet, uint8_t>;
//using lcdPwm = SoftPWM<fetPin>;

#ifdef USE_BUTTON
using buttonPin = AVR::Pin<PortB, 0>;
using button0 = Button<0, buttonPin>;
using buttonController = ButtonController<button0>;
#else
using debugPin = AVR::Pin<PortB, 0>; 
#endif

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

using LcdData = AVR::PinSet<AVR::UsePgmTable, LcdDB4, LcdDB5, LcdDB6, LcdDB7>;

using lcd = LCD::HD44780Port<LcdData, LcdRS, LcdRW, LcdE, LCD::Lcd2x16>;
using lcdStream = BufferedStream<lcd, 64, std::lineTerminator<std::LF>>;

using systemClock = AVR::Timer8Bit<0>;
using systemTimer = AlarmTimer<systemClock>;

using systemConstantRate = ConstantRateAdapter<1, void, AVR::ISR::Timer<0>::CompareA, systemTimer, 
//lcdPwm,
#ifdef USE_BUTTON
                                                buttonController, 
#endif
rotaryEncoder>;

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

constexpr TWI::Address address{0x59_B};
using i2c = TWI::Slave<0, address, lcd::param_type::rows * lcd::param_type::cols>;

//using isrReg = IsrRegistrar<spiInput, terminalDevive::RxHandler, terminalDevive::TxHandler, i2c>; 
using isrReg = IsrRegistrar<terminalDevive::RxHandler, terminalDevive::TxHandler, i2c>; 

//struct Spi0handler: public EventHandler<EventType::Spi0> {
//    static bool process(std::byte v) {
//        Util::put<terminalDevive, true>(v);
//        return true;
//    }
//};

struct Timerhandler: public EventHandler<EventType::Timer> {
    static bool process(std::byte){
        using namespace std::literals::quantity;
        
        static uint8_t counter = 0;
        ++counter;
        
        std::percent v = std::scale(counter % 10u, 0, 9);
//        std::percent v = 80_ppc;
        
        lcdPwm::pwm(v, 0);
        blinker::tick();

        lcd::home();
//        lcdcout << "R: " << rotaryEncoder::value();
        std::out<lcdStream>("R: "_pgm, rotaryEncoder::value());
        
        return true;
    }
};

int main() {
    isrReg::init();
    blinker::init();
    systemTimer::init(AVR::TimerMode::CTCNoInt);
    terminalDevive::init<19200>();
    lcdPwm::init();    
    lcd::init();
    i2c::init();
//    spiInput::init<AVR::SpiSlave<>>();
    spiInput::init();
#ifdef USE_BUTTON
    buttonController::init();
#else
    debugPin::dir<AVR::Output>();
    debugPin::off();
#endif
    rotaryEncoder::init();
    
    auto title = "UniMaxi (HW 0.2) m328 (Spi Uart Lcd) 0.93"_pgm;
    
    std::outl<terminal>(title);
    std::outl<lcdStream>(title);

    std::outl<terminal>(Config());
    
    systemTimer::create(500_ms, AlarmFlags::Periodic);
    
//    using handler = EventHandlerGroup<Spi0handler, Timerhandler>;
    using handler = EventHandlerGroup<Timerhandler>;
    
    std::fill(i2c::registers().begin(), i2c::registers().end(), std::byte{' '});
    
    // todo: usart ohne Interrupt
    // todo: spi ohne Interrupt
    // todo: twi ohne Interrupt
    
    {
        Scoped<EnableInterrupt<>> interruptEnabler;
        EventManager::run2<handler>([](){
            systemClock::periodic<systemClock::flags_type::ocfa>([](){
                systemTimer::rateProcess();
                systemConstantRate::rateTick();
                debugPin::toggle();
            });
            systemConstantRate::periodic();
            lcdPwm::freeRun();
            lcdStream::periodic();
            spiInput::whenReady([](std::byte b){
                terminalDevive::put(b);
            });
//            if (spiInput::leak()) {
//                blinker::failure0();
//            }
            if (i2c::isChanged()) {
                i2c::changed(false);
                lcd::setPosition(LCD::Row{1}, LCD::Column{0});
                lcd::put(i2c::registers()[0]);
            }
            if (EventManager::unprocessedEvent()) {
                blinker::failure1();
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

ISR(TWI_vect) {
    isrReg::isr<AVR::ISR::Twi<0>>();
}
//ISR(SPI_STC_vect) {
//    isrReg::isr<AVR::ISR::Spi<0>::Stc>();
//}
ISR(USART_RX_vect) {
    isrReg::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART_UDRE_vect){
    isrReg::isr<AVR::ISR::Usart<0>::UDREmpty>();
}
