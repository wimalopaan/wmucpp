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
#include "appl/blink.h"
#include "appl/ledflash.h"
#include "std/chrono.h"

#include "console.h"

constexpr bool use4x4    = false;
constexpr bool useRing60 = true;

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using iRPin    = AVR::Pin<PortB, 0>;
using leds1Pin = AVR::Pin<PortB, 1>;
using leds2Pin = AVR::Pin<PortB, 2>;
using ppm1Pin  = AVR::Pin<PortB, 3>;

using adc = AdcController<AVR::Adc<0>, 0>;

using rxdPin         = AVR::Pin<PortD, 0>;
using txdPin         = AVR::Pin<PortD, 1>;
using dcfPin         = AVR::Pin<PortD, 2>;
using ppm2Pin        = AVR::Pin<PortD, 3>;
using powerSwitchPin = AVR::Pin<PortD, 4>;
using spiClockPin    = AVR::Pin<PortD, 5>;
using spiDataPin     = AVR::Pin<PortD, 6>;
using ledPin         = AVR::Pin<PortD, 7>;

using led = WS2812<1, ledPin, ColorSequenceGRB>;
using Color = led::color_type;

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

static constexpr uint16_t analogBrightnessMaximum = 400;

static constexpr uint32_t secondsPerDay = (uint32_t)60 * 60 * 24;

static constexpr auto title = "Clock Uni 01"_pgm;
}

using statusLed = LedFlash<led>;

using leds1 = WS2812<16, leds1Pin, ColorSequenceGRB>;
using leds2 = WS2812<60, leds2Pin, ColorSequenceGRB>;

using terminal = AVR::Usart<0, void>;

using systemTimer = AVR::Timer8Bit<0>; // timer 0
using alarmTimer  = AlarmTimer<systemTimer>;

using dcfDecoder = DCF77<dcfPin, Config::Timer::frequency, EventManager, true>;

using systemConstantRate = ConstantRateAdapter<void, AVR::ISR::Timer<0>::CompareA, alarmTimer, dcfDecoder>;

using isrRegistrar = IsrRegistrar<systemConstantRate, terminal::RxHandler, terminal::TxHandler>;

template<typename Leds, 
         const typename Leds::color_type& tickColor,
         const typename Leds::color_type& hourColor,
         const typename Leds::color_type& minuteColor,
         const typename Leds::color_type& secondColor
         >
class TimerDisplay60 {
    TimerDisplay60() = delete;
public:
    static constexpr uint8_t size = 60;
    static constexpr uint8_t hours = 12;
    static constexpr uint8_t minuteTick = 5;
    
    static_assert(Leds::size == size, "wrong number of leds");

    static void init() {
        Leds::init();
    }
    
    template<typename Clock>
    static void set(const Clock& clock) {
        DateTime::TimeTm t = clock.dateTime();
        
        // todo: minimale Helligkeit, so dass es nicht ganz auf null geht
        Leds::template set<false>(Constant::cOff);
        for(uint8_t i = 0; i < size; ++i) {
            if ((i % minuteTick) == 0) {
                Leds::template set<false>(i, tickColor * brightness());
            }
        }
        
        Leds::template set<false>(((t.hours().value % hours * minuteTick) + (t.minutes().value * minuteTick) / size) % size, hourColor * brightness());
        Leds::template add<false>(t.minutes().value, minuteColor * brightness());
        Leds::template add<false>(t.seconds().value, secondColor * brightness());
        Leds::write();
    }
    
    static std::percent& brightness() {
        static std::percent p = 100_ppc;
        return p;
    }
};

// todo: Farbein einbauen
template<typename Leds>
class TimerDisplay4x4 {
    TimerDisplay4x4() = delete;
public:
    static constexpr uint8_t size = 16;
    static_assert(Leds::size == size, "wrong number of leds");

    static void init() {
        Leds::init();
    }

    template<typename Clock>
    static void set(const Clock& clock) {
        DateTime::TimeTm t = clock.dateTime();
        uint8_t min1   = t.minutes().value % 10;
        uint8_t min10  = t.minutes().value / 10;
        uint8_t hour1  = t.hours().value % 10;
        uint8_t hour10 = t.hours().value / 10;
        
        setNibble<0>(min1);
        setNibble<1>(min10);
        setNibble<2>(hour1);
        setNibble<3>(hour10);
        
        Leds::write();
    }
    
    template<uint8_t N>
    static void setNibble(uint8_t v) {
        if constexpr((N % 2) == 0) {
            for(uint8_t i = 0; i < 4; ++i) {
                if (v & (1 << i)) {
                    Leds::template set<false>(4 * N + i, Constant::cBlue);
                }
                else {
                    Leds::template set<false>(4 * N + i, Constant::cOff);
                }
            }
        }
        else {
            for(uint8_t i = 0; i < 4; ++i) {
                if (v & (1 << i)) {
                    Leds::template set<false>(4 * (N + 1) - 1 - i, Constant::cBlue);
                }
                else {
                    Leds::template set<false>(4 * (N + 1) - 1 - i, Constant::cOff);
                }
            }
        }
    }
    static std::percent& brightness() {
        static std::percent p = 100_ppc;
        return p;
    }
};

using display = TimerDisplay4x4<leds1>;
using display2 = TimerDisplay60<leds2, Constant::cWhiteLow, Constant::cBlue, Constant::cGreen, Constant::cRed>;

namespace std {
std::basic_ostream<terminal> cout;
std::lineTerminator<CRLF> endl;
}

const auto blinkTimer = alarmTimer::create(100_ms, AlarmFlags::Periodic);
const auto secondsTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);
const auto preStartTimer = alarmTimer::create(3000_ms, AlarmFlags::OneShot | AlarmFlags::Disabled);

std::chrono::system_clock<> systemClock;

std::percent brightness = 100_ppc;


// todo: für ReSync erweitern: falls Resync nicht erfolgreich -> Zeit beibehalten und ReSyncCounter irgendwie anzeigen (Farbe ander 12).
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
                statusLed::steadyColor(Constant::cOff);
                powerSwitchPin::off();
                break;
            case State::Start:
                std::cout << "S: Start"_pgm << std::endl;
                statusLed::steadyColor(Constant::cBlue * brightness);
                powerSwitchPin::off();
                break;
            case State::Sync1:
                std::cout << "S: Sync1"_pgm << std::endl;
                statusLed::steadyColor(Constant::cYellow * brightness);
                powerSwitchPin::off();
                break;
            case State::Sync2:
                std::cout << "S: Sync2"_pgm << std::endl;
                statusLed::steadyColor(Constant::cCyan * brightness);
                powerSwitchPin::off();
                break;
            case State::Clock:
                std::cout << "S: Clock"_pgm << std::endl;
                statusLed::steadyColor(Constant::cGreen * brightness);
                systemClock = std::chrono::system_clock<>::from(dcfDecoder::dateTime());
                powerSwitchPin::on();
                break;
            case State::Error:
                std::cout << "S: Error"_pgm << std::endl;
                statusLed::steadyColor(Constant::cMagenta * brightness);
                powerSwitchPin::off();
                break;
            }
        }
    }        
};

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(uint8_t timer) {
        if (timer == *blinkTimer) {
            statusLed::tick(brightness);
        }
        else if (timer == *secondsTimer) {
            std::cout << "light: "_pgm << brightness << std::endl;
            if (systemClock) {
                if constexpr(use4x4) {
                    display::set(systemClock);
                }
                if constexpr(useRing60) {
                    display2::set(systemClock);
                }
                
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
        statusLed::flash(Constant::cRed * brightness, 1);
        return true;
    }  
};
struct DCFReceive1Handler : public EventHandler<EventType::DCFReceive1> {
    static bool process(uint8_t) {
        statusLed::flash(Constant::cRed * brightness, 2);
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
    
    isrRegistrar::init();
    alarmTimer::init();
    
    powerSwitchPin::dir<AVR::Output>();    
    powerSwitchPin::off();    
    
    terminal::init<19200>();
    
    statusLed::init();
    
    if constexpr(use4x4) {
        display::init();
    }
    if constexpr(useRing60) {
        display2::init();
    }

    adc::init();
    
    {
        Scoped<EnableInterrupt> ei;
        std::cout << Constant::title << std::endl;
        alarmTimer::start(*preStartTimer);
        EventManager::run2<allEventHandler>([](){
            adc::periodic();
            brightness = std::scale(adc::value(0), adc::value_type(0), adc::value_type(Constant::analogBrightnessMaximum));
            if constexpr(useRing60) {
                display2::brightness() = brightness;
            }
            if constexpr(use4x4) {
                display::brightness() = brightness;
            }

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
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif
