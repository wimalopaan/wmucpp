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

// 20MHz full-swing
// sudo avrdude -p atmega1284P -P usb -c avrisp2 -U lfuse:w:0xf7:m -U hfuse:w:0xd1:m -U efuse:w:0xfc:m

#include <stdlib.h>
#include <util/eu_dst.h>

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/adc.h"
#include "hal/event.h"
#include "hal/constantrate.h"
#include "hal/alarmtimer.h"
#include "hal/adccontroller.h"
#include "external/ws2812.h"
#include "external/dcf77.h"
#include "external/lm35.h"
#include "std/chrono.h"
#include "appl/blink.h"
#include "appl/ledflash.h"
#include "appl/wordclock.h"
#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using adc = AdcController<AVR::Adc<0>, 0, 1>;

using iRPin          = AVR::Pin<PortA, 2>;
using dcfPin         = AVR::Pin<PortA, 3>;
using powerSwitchPin = AVR::Pin<PortA, 4>;
using ledsPin        = AVR::Pin<PortD, 4>;
using debugPin       = AVR::Pin<PortD, 7>;

namespace Constant {
static constexpr uint8_t brightness = 255;
//static constexpr Color cOff{0};
//static constexpr Color cRed{Red{brightness}};
//static constexpr Color cBlue{Blue{std::min((int)std::numeric_limits<uint8_t>::max(), 2 * brightness)}};
//static constexpr Color cGreen{Green{brightness / 2}};
//static constexpr Color cYellow{Red{brightness}, Green{brightness}, Blue{0}};
//static constexpr Color cMagenta{Red{brightness}, Green{0}, Blue{brightness}};
//static constexpr Color cCyan{Red{0}, Green{brightness}, Blue{brightness}};
//static constexpr Color cWhite{Red{brightness}, Green{brightness}, Blue{brightness}};
//static constexpr Color cWhiteLow{Red{brightness / 10}, Green{brightness / 10}, Blue{brightness / 10}};

static constexpr uint16_t analogBrightnessMaximum = 1023;

static constexpr uint32_t secondsPerDay = (uint32_t)60 * 60 * 24;

static constexpr auto title = "Wordclock 01"_pgm;
}

using leds = WS2812<110, ledsPin, ColorSequenceGRB>;

using temp = LM35<adc, 1>;

using terminal = AVR::Usart<1, void>;

using systemTimer = AVR::Timer8Bit<0>; // timer 0
using alarmTimer  = AlarmTimer<systemTimer>;

using dcfDecoder = DCF77<dcfPin, Config::Timer::frequency, EventManager, true>;

using systemConstantRate = ConstantRateAdapter<void, AVR::ISR::Timer<0>::CompareA, alarmTimer, dcfDecoder>;

using isrRegistrar = IsrRegistrar<systemConstantRate, terminal::RxHandler, terminal::TxHandler>;


namespace std {
std::basic_ostream<terminal> cout;
std::lineTerminator<CRLF> endl;
}

const auto blinkTimer = alarmTimer::create(100_ms, AlarmFlags::Periodic);
const auto secondsTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);
const auto preStartTimer = alarmTimer::create(3000_ms, AlarmFlags::OneShot | AlarmFlags::Disabled);

std::chrono::system_clock<> systemClock;

std::percent brightness = 100_ppc;

struct ClockStateMachine {
    enum class State : uint8_t {PreStart, Start, Sync1, Sync2, Clock, Error};
    enum class Event : uint8_t {Reset, Start, DCFSync, DCFDecode, DCFError, ReSync};
    static void process(Event event) {
        static State state = State::PreStart;
        State newState = state;
        switch (state) {
        case State::PreStart:
            if (event == Event::Start) {
                newState = State::Start;
            }
            break;
        case State::Start:
            if (event == Event::DCFSync) {
                newState = State::Sync1;
            }
            else if (event == Event::Reset) {
                newState = State::PreStart;
            }
            break;
        case State::Sync1:
            if (event == Event::DCFDecode) {
                newState = State::Sync2;
            }
            else if (event == Event::Reset) {
                newState = State::PreStart;
            }
            else if (event == Event::DCFError) {
                newState = State::Error;
            }
            break;
        case State::Sync2:
            if (event == Event::DCFDecode) {
                newState = State::Clock;
            }
            else if (event == Event::Reset) {
                newState = State::PreStart;
            }
            else if (event == Event::DCFError) {
                newState = State::Error;
            }
            break;
        case State::Clock:
            if (event == Event::ReSync) {
                newState = State::Start;
            }
            else if (event == Event::ReSync) {
                newState = State::PreStart;
            }
            break;
        case State::Error:
            if (event == Event::DCFSync) {
                newState = State::Start;
            }
            break;
        }
        if (newState != state) {
            state = newState;
            switch (state) {
            case State::PreStart:
                std::cout << "S: PreStart"_pgm << std::endl;
                powerSwitchPin::off();
                break;
            case State::Start:
                std::cout << "S: Start"_pgm << std::endl;
                powerSwitchPin::off();
                break;
            case State::Sync1:
                std::cout << "S: Sync1"_pgm << std::endl;
                powerSwitchPin::off();
                break;
            case State::Sync2:
                std::cout << "S: Sync2"_pgm << std::endl;
                powerSwitchPin::off();
                break;
            case State::Clock:
                std::cout << "S: Clock"_pgm << std::endl;
                systemClock = std::chrono::system_clock<>::from(dcfDecoder::dateTime());
                powerSwitchPin::on();
                break;
            case State::Error:
                std::cout << "S: Error"_pgm << std::endl;
                powerSwitchPin::off();
                break;
            }
        }
    }        
};

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(uint8_t timer) {
        if (timer == *blinkTimer) {
            debugPin::toggle();
        }
        else if (timer == *secondsTimer) {
            std::cout << "light: "_pgm << brightness << std::endl;
            std::cout << "temp: "_pgm << temp::temperature() << std::endl;
            if (systemClock) {
                
                std::cout << systemClock.dateTime() << std::endl;
                
                if ((systemClock.value() % Constant::secondsPerDay) == 0) {
                    systemClock = -1;
                    ClockStateMachine::process(ClockStateMachine::Event::ReSync);
                    std::cout << "ReSync"_pgm << std::endl;
                }
            }
            systemClock.tick();            
        }
        else if (timer == *preStartTimer) {
            ClockStateMachine::process(ClockStateMachine::Event::Start);
        }
        return true;
    }
};
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
struct DCFReceive0Handler : public EventHandler<EventType::DCFReceive0> {
    static bool process(uint8_t) {
        return true;
    }  
};
struct DCFReceive1Handler : public EventHandler<EventType::DCFReceive1> {
    static bool process(uint8_t) {
        return true;
    }  
};
struct DCFDecodeHandler : public EventHandler<EventType::DCFDecode> {
    static bool process(uint8_t) {
        ClockStateMachine::process(ClockStateMachine::Event::DCFDecode);
        return true;
    }  
};
struct DCFSyncHandler : public EventHandler<EventType::DCFSync> {
    static bool process(uint8_t) {
        ClockStateMachine::process(ClockStateMachine::Event::DCFSync);
        return true;
    }  
};
struct DCFErrorHandler : public EventHandler<EventType::DCFError> {
    static bool process(uint8_t) {
        ClockStateMachine::process(ClockStateMachine::Event::DCFError);
        return true;
    }  
};
struct DCFParityHandler : public EventHandler<EventType::DCFParityError> {
    static bool process(uint8_t) {
        ClockStateMachine::process(ClockStateMachine::Event::DCFError);
        return true;
    }  
};

using allEventHandler = EventHandlerGroup<TimerHandler, UsartFeHandler, UsartUpeHandler, UsartDorHandler, Usart0Handler,
                                        DCFReceive0Handler, DCFReceive1Handler, DCFDecodeHandler, DCFSyncHandler, DCFErrorHandler, DCFParityHandler>;

int main()
{   
    set_zone(ONE_HOUR); // europe central time
    set_dst(eu_dst);
    
    debugPin::dir<AVR::Output>();
    debugPin::off();
    
    isrRegistrar::init();
    alarmTimer::init();
    
    powerSwitchPin::dir<AVR::Output>();    
    powerSwitchPin::off();    
    
    terminal::init<19200>();
    
    adc::init();
    
    {
        Scoped<EnableInterrupt> ei;
        std::cout << Constant::title << std::endl;
        alarmTimer::start(*preStartTimer);
        EventManager::run2<allEventHandler>([](){
            adc::periodic();
            brightness = std::scale(adc::value(0), adc::value_type(0), adc::value_type(Constant::analogBrightnessMaximum));

            systemConstantRate::periodic();
            if (EventManager::unprocessedEvent()) {
                EventManager::unprocessedEvent() = false;
            }
            if (EventManager::leakedEvent()) {
                EventManager::leakedEvent() = false;
            }
        });
    }
}

ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}
ISR(USART1_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
}
ISR(USART1_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif
