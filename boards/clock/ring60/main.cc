/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

// 20Mhz extern
// sudo avrdude -p atmega88p -P usb -c avrisp2 -U lfuse:w:0xd0:m -U hfuse:w:0xdf:m -U efuse:w:0xf9:m

// 8Mhz RC intern
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

#define NDEBUG
#define USE_DEPRECATED

#include <stdlib.h>
#include <chrono>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <util/eu_dst.h>
#pragma GCC diagnostic pop

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/adc.h"
#include "mcu/avr/clock.h"
#include "hal/event.h"
#include "hal/constantrate.h"
#include "hal/alarmtimer.h"
#include "hal/adccontroller.h"
#include "external/ws2812.h"
#include "external/dcf77.h"
#include "appl/blink.h"
#include "appl/ledflash.h"
#include "appl/timerdisplayring60.h"
#include "appl/timerdisplay4x4.h"

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

using adc = AdcController<AVR::Adc<0, AVR::Resolution<10>>, 0>;

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

using leds1 = WS2812<16, leds2Pin, ColorSequenceGRB>;
using leds2 = WS2812<60, leds1Pin, ColorSequenceGRB>;

using terminal = AVR::Usart<0, void>;

using systemTimer = AVR::Timer16Bit<1>; // timer 1

struct LocalConfig {
    static constexpr AVR::Util::TimerSetupData tsd = AVR::Util::caculateForExactFrequencyAbove<systemTimer>(Config::Timer::frequency);
    static_assert(tsd, "wrong timer parameter");
    static constexpr std::milliseconds reso = std::duration_cast<std::milliseconds>(1 / tsd.f);
    static constexpr std::hertz exactFrequency = tsd.f;
};


using alarmTimer  = AlarmTimer<systemTimer, UseEvents<true>, LocalConfig::reso>;

using dcfDecoder = DCF77<dcfPin, LocalConfig::exactFrequency, EventManager, true>;

using oscillator = AVR::Clock<LocalConfig::exactFrequency>;

using systemConstantRate = ConstantRateAdapter<1, void, AVR::ISR::Timer<1>::CompareA, alarmTimer, dcfDecoder, oscillator>;

using isrRegistrar = IsrRegistrar<systemConstantRate, terminal::RxHandler, terminal::TxHandler>;

using display = TimerDisplay4x4<leds1, Constant::cBlue>;
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
    static bool process(std::byte b) {
        auto timer = std::to_integer<uint7_t>(b);
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
        ClockStateMachine::process(ClockStateMachine::Event::DCFDecode);
        
        static uint8_t intervall = 0;
                
        if (auto m = oscillator::actualMeasurementDuration(dcfDecoder::dateTime()); m && *m >= oscillator::mIntervalls[intervall]) {
            oscillator::referenceTime(dcfDecoder::dateTime());
            std::cout << "Delta: "_pgm << oscillator::delta() << std::endl;
            std::cout << "Durat: "_pgm << oscillator::lastMeasurementDuration().value << std::endl;
            
            if (auto delta = oscillator::delta(); delta != 0) {
                if ((abs(delta) >= thresh)) {
                    if (delta > 0) {
                        oscillator::adjust(-1);
                    }
                    else {
                        oscillator::adjust(1);
                    }
                    std::cout << "OSCCAL: "_pgm << oscillator::calibration() << std::endl;
                }
                else {
                    intervall = std::min((uint8_t)(intervall + 1), (uint8_t)(oscillator::mIntervalls.size - 1));
                    std::cout << "interval: "_pgm << oscillator::mIntervalls[intervall].value << std::endl;
                }
            }
        }
        return true;
    }  
    inline static constexpr int32_t thresh = 10;
};
struct DCFSyncHandler : public EventHandler<EventType::DCFSync> {
    static bool process(std::byte) {
        ClockStateMachine::process(ClockStateMachine::Event::DCFSync);
        return true;
    }  
};
struct DCFErrorHandler : public EventHandler<EventType::DCFError> {
    static bool process(std::byte) {
        ClockStateMachine::process(ClockStateMachine::Event::DCFError);
        return true;
    }  
};
struct DCFParityHandler : public EventHandler<EventType::DCFParityError> {
    static bool process(std::byte) {
        ClockStateMachine::process(ClockStateMachine::Event::DCFError);
        return true;
    }  
};

using allEventHandler = EventHandlerGroup<TimerHandler, UsartFeHandler, UsartUpeHandler, UsartDorHandler, Usart0Handler,
                                        DCFReceive0Handler, DCFReceive1Handler, DCFDecodeHandler, DCFSyncHandler, DCFErrorHandler, DCFParityHandler>;

int main() {   
    MCU::Ressource::Registrar<systemConstantRate>::init();
    isrRegistrar::init();
    
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

    systemTimer::template prescale<LocalConfig::tsd.prescaler>();
    systemTimer::template ocra<LocalConfig::tsd.ocr - 1>();
    systemTimer::mode(AVR::TimerMode::CTC);
    systemTimer::start();

    adc::init();
    
    {
        Scoped<EnableInterrupt<>> ei;
        std::cout << Constant::title << std::endl;
        alarmTimer::start(*preStartTimer);
        EventManager::run2<allEventHandler>([](){
            adc::periodic();
            // todo: move to BrightnesController
//            brightness = std::scale(adc::value(0), adc::value_type(0), adc::value_type(Constant::analogBrightnessMaximum));
//            brightness = std::max(brightness, 1_ppc);
            
            if constexpr(useRing60) {
                display2::brightness(brightness);
            }
            if constexpr(use4x4) {
                display::brightness(brightness);
            }

            systemConstantRate::periodic();
            if (EventManager::unprocessedEvent()) {
                statusLed::flash(Constant::cMagenta, 10);
            }
            if (EventManager::leakedEvent()) {
                statusLed::flash(Constant::cYellow, 10);
            }
        });
    }
}

ISR(TIMER1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
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
