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

// sudo avrdude -p atmega88p -P usb -c avrisp2 -U lfuse:w:0xd0:m -U hfuse:w:0xdf:m -U efuse:w:0xf9:m

#define NDEBUG

#include <stdlib.h>

#include "../../include/universal01.h"

#include "hal/event.h"
#include "hal/constantrate.h"
#include "hal/alarmtimer.h"
#include "appl/ledflash.h"
#include "appl/clockstatemachine.h"
#include "appl/timerdisplay4x4.h"
#include "std/chrono.h"
#include "console.h"

namespace Constant {
static constexpr uint8_t brightness = 255;
static constexpr Color cOff{0};
static constexpr Color cRed{Red{brightness}};
static constexpr Color cBlue{Blue{std::min((int)std::numeric_limits<uint8_t>::max(), 2 * brightness)}};
static constexpr Color cGreen{Green{brightness / 2}};
static constexpr Color cYellow{Red{brightness}, Green{brightness}, Blue{0}};
static constexpr Color cMagenta{Red{brightness}, Green{0}, Blue{brightness}};
static constexpr Color cCyan{Red{0}, Green{brightness}, Blue{brightness}};
//static constexpr Color cWhite{Red{brightness}, Green{brightness}, Blue{brightness}};
//static constexpr Color cWhiteLow{Red{brightness / 10}, Green{brightness / 10}, Blue{brightness / 10}};

static constexpr uint16_t analogBrightnessMaximum = 400;

static constexpr uint32_t secondsPerDay = (uint32_t)60 * 60 * 24;

static constexpr auto title = "Clock 4x4"_pgm;
}

using statusLed = LedFlash<led>;

using leds1 = WS2812<16, leds1Pin, ColorSequenceGRB>;

using terminalDevive = AVR::Usart<0, void>;
using terminal = std::basic_ostream<terminalDevive>;

using systemTimer = AVR::Timer8Bit<0>; // timer 0
using alarmTimer  = AlarmTimer<systemTimer>;

using dcfDecoder = DCF77<dcfPin, Config::Timer::frequency, EventManager, true>;

using systemConstantRate = ConstantRateAdapter<void, AVR::ISR::Timer<0>::CompareA, alarmTimer, dcfDecoder>;

using isrRegistrar = IsrRegistrar<systemConstantRate, terminalDevive::RxHandler, terminalDevive::TxHandler>;

using display = TimerDisplay4x4<leds1, Constant::cBlue>;

const auto blinkTimer = alarmTimer::create(100_ms, AlarmFlags::Periodic);
const auto secondsTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);
const auto preStartTimer = alarmTimer::create(3000_ms, AlarmFlags::OneShot | AlarmFlags::Disabled);

std::chrono::system_clock<> systemClock;

std::percent brightness = 100_ppc;

template<typename PowerPin = void>
struct StateManager{
    using State = typename ClockStateMachine::State;
    static void enter(State state) {
        switch(state) {
        case State::PreStart:
            std::outl<terminal>("S: PreStart"_pgm);
            statusLed::steadyColor(Constant::cOff);
            if constexpr(!std::is_same<PowerPin, void>::value) {
                powerSwitchPin::off();
            }
            break;
        case State::Start:
            std::outl<terminal>("S: Start"_pgm);
            statusLed::steadyColor(Constant::cBlue * brightness);
            if constexpr(!std::is_same<PowerPin, void>::value) {
                powerSwitchPin::off();
            }
            break;
        case State::Sync1:
            std::outl<terminal>("S: Sync1"_pgm);
            statusLed::steadyColor(Constant::cYellow * brightness);
            if constexpr(!std::is_same<PowerPin, void>::value) {
                powerSwitchPin::off();
            }
            break;
        case State::Sync2:
            std::outl<terminal>("S: Sync2"_pgm);
            statusLed::steadyColor(Constant::cCyan * brightness);
            if constexpr(!std::is_same<PowerPin, void>::value) {
                powerSwitchPin::off();
            }
            break;
        case State::Sync3:
            std::outl<terminal>("S: Sync3"_pgm);
            statusLed::steadyColor(Constant::cRed * brightness);
            if constexpr(!std::is_same<PowerPin, void>::value) {
                powerSwitchPin::off();
            }
            break;
        case State::Clock:
            std::out<terminal>("S: Clock"_pgm);
            statusLed::steadyColor(Constant::cGreen * brightness);
            systemClock = std::chrono::system_clock<>::from(dcfDecoder::dateTime());
            if constexpr(!std::is_same<PowerPin, void>::value) {
                powerSwitchPin::on();
            }
            break;
        case State::Error:
            std::outl<terminal>("S: Error"_pgm);
            statusLed::steadyColor(Constant::cMagenta * brightness);
            if constexpr(!std::is_same<PowerPin, void>::value) {
                powerSwitchPin::off();
            }
            break;
        }
    }        
};

using clockFSM = ClockStateMachine::Machine<StateManager<powerSwitchPin>>;

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(std::byte b) {
        auto timer = std::to_integer<uint7_t>(b);
        if (timer == *blinkTimer) {
            statusLed::tick(brightness);
        }
        else if (timer == *secondsTimer) {
            std::outl<terminal>("light: "_pgm, brightness);
            if (systemClock) {
                display::set(systemClock);
                
                std::outl<terminal>(systemClock.dateTime());
                
                if ((systemClock.value() % Constant::secondsPerDay) == 0) {
                    systemClock = -1;
                    clockFSM::process(ClockStateMachine::Event::ReSync);
                    std::outl<terminal>("ReSync"_pgm);
                }
            }
            systemClock.tick();            
        }
        else if (timer == *preStartTimer) {
            clockFSM::process(ClockStateMachine::Event::Start);
        }
        return true;
    }
};
struct Usart0Handler : public EventHandler<EventType::UsartRecv0> {
    static bool process(std::byte) {
        return true;
    }
};
struct UsartFeHandler : public EventHandler<EventType::UsartFe> {
    static bool process(std::byte) {
        return true;
    }
};
struct UsartUpeHandler : public EventHandler<EventType::UsartUpe> {
    static bool process(std::byte) {
        return true;
    }
};
struct UsartDorHandler : public EventHandler<EventType::UsartDor> {
    static bool process(std::byte) {
        return true;
    }
};
struct DCFReceive0Handler : public EventHandler<EventType::DCFReceive0> {
    static bool process(std::byte) {
        statusLed::flash(Constant::cRed * brightness, 1);
        return true;
    }  
};
struct DCFReceive1Handler : public EventHandler<EventType::DCFReceive1> {
    static bool process(std::byte) {
        statusLed::flash(Constant::cRed * brightness, 2);
        return true;
    }  
};
struct DCFDecodeHandler : public EventHandler<EventType::DCFDecode> {
    static bool process(std::byte) {
        clockFSM::process(ClockStateMachine::Event::DCFDecode);
        return true;
    }  
};
struct DCFSyncHandler : public EventHandler<EventType::DCFSync> {
    static bool process(std::byte) {
        clockFSM::process(ClockStateMachine::Event::DCFSync);
        return true;
    }  
};
struct DCFErrorHandler : public EventHandler<EventType::DCFError> {
    static bool process(std::byte) {
        clockFSM::process(ClockStateMachine::Event::DCFError);
        return true;
    }  
};
struct DCFParityHandler : public EventHandler<EventType::DCFParityError> {
    static bool process(std::byte) {
        clockFSM::process(ClockStateMachine::Event::DCFError);
        return true;
    }  
};

using allEventHandler = EventHandlerGroup<TimerHandler, UsartFeHandler, UsartUpeHandler, UsartDorHandler, Usart0Handler,
                                        DCFReceive0Handler, DCFReceive1Handler, DCFDecodeHandler, DCFSyncHandler, DCFErrorHandler, DCFParityHandler>;

int main(){   
    isrRegistrar::init();
    alarmTimer::init();
    powerSwitchPin::dir<AVR::Output>();    
    powerSwitchPin::off();    
    terminalDevive::init<19200>();
    statusLed::init();
    display::init();
    adc::init();
    
    {
        Scoped<EnableInterrupt> ei;
        std::outl<terminal>(Constant::title);
        alarmTimer::start(*preStartTimer);
        EventManager::run2<allEventHandler>([](){
            adc::periodic();
            brightness = std::scale(adc::value(0), adc::value_type(0), adc::value_type(Constant::analogBrightnessMaximum));
  
            display::brightness = brightness;

            systemConstantRate::periodic();
            if (EventManager::unprocessedEvent()) {
                EventManager::unprocessedEvent() = false;
                statusLed::flash(Constant::cMagenta, 10);
            }
            if (EventManager::leakedEvent()) {
                EventManager::leakedEvent() = false;
                statusLed::flash(Constant::cYellow, 10);
            }
        });
    }
}

ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}
ISR(USART_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, ',', file, ',', line);
    while(true) {}
}
#endif
