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

#include "mcu/avr/clock.h"
#include "hal/event.h"
#include "hal/constantrate.h"
#include "hal/alarmtimer.h"
#include "hal/softspimaster.h"
#include "std/chrono.h"
#include "external/irmp.h"
#include "appl/ledflash.h"
#include "appl/clockstatemachine.h"
#include "appl/timerdisplay4x4.h"
#include "appl/blink.h"
#include "appl/timebcdserial.h"
#include "appl/timerdisplay4x4.h"
#include "appl/timerdisplayring60.h"
#include "appl/radioclock.h"
#include "appl/wordclock.h"
#include "appl/fonts.h"
#include "appl/shiftdisplay.h"

#include "console.h"

static constexpr bool useLight = true;
static constexpr bool useIR = false;
static constexpr bool useText = false;
static constexpr bool useRCCalibration = false;
static constexpr bool useUsart = false;

using d0_ss_Pin = leds2Pin;
using d1_data_Pin = spiDataPin;
using displayLedsPin = leds1Pin;
using d2_clck_Pin = spiDataPin;
//using d3_Pin = AVR::Pin<PortD, 3>;

using display4x4Leds = WS2812<16, displayLedsPin, ColorSequenceGRB>;
using display60Leds = WS2812<60, displayLedsPin, ColorSequenceGRB>;

using led = WS2812<1, ledPin, ColorSequenceGRB>;
using Color = display4x4Leds::color_type;
//using Color = led::color_type;

using spi = SoftSpiMaster<d1_data_Pin, d2_clck_Pin>;

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

static constexpr uint32_t secondsPerHour= (uint32_t)60 * 60;
static constexpr uint32_t secondsPerDay = secondsPerHour * 24;

static constexpr auto title = "SW = 4x4; HW = Universal01"_pgm;

static constexpr uint8_t textLength = std::max(uint8_t(64), title.size);
} // !Constant

using displayNixie  = TimeBCDSerial<spi>;
using display4x4    = TimerDisplay4x4<display4x4Leds, Constant::cRed>;
using displayRing60 = TimerDisplay60<display60Leds, Constant::cWhiteLow, Constant::cBlue, Constant::cGreen, Constant::cRed>;
using displayWC     = WordclockDisplay<d0_ss_Pin, ColorSequenceGRB>;
using display       = display4x4;

using shifttext = std::conditional<useText, ShiftDisplay<displayWC::leds, displayWC::mColumns, displayWC::mRows, Constant::textLength, Font<5,7>>, void>::type;
//using shifttext = ShiftDisplay<displayWC::leds, displayWC::mColumns, displayWC::mRows, Constant::textLength, Font<5,7>>;

struct NullDevice {
    template<uint16_t B>
    static void init() {}
    static bool put(std::byte) {return true;}
};


//using terminalDevice = NullDevice;
using terminalDevice = AVR::Usart<0, void>;
using terminal = std::basic_ostream<terminalDevice>;

namespace std {
std::basic_ostream<terminalDevice> cout;
std::lineTerminator<CRLF> endl;
}

using statusLed = LedFlash<led>;

using systemTimer = AVR::Timer16Bit<1>;

struct LocalConfig {
    static constexpr AVR::Util::TimerSetupData tsd = AVR::Util::caculateForExactFrequencyAbove<systemTimer>(Config::Timer::frequency);
    static_assert(tsd, "wrong timer parameter");
    static constexpr std::milliseconds reso = std::duration_cast<std::milliseconds>(1 / tsd.f);
    static constexpr std::hertz exactFrequency = tsd.f;
};

using alarmTimer  = AlarmTimer<systemTimer, LocalConfig::reso>;

using dcfDecoder = DCF77<dcfPin, LocalConfig::exactFrequency, EventManager, true>;

using oscillator = AVR::Clock<LocalConfig::exactFrequency>;

using systemConstantRate = ConstantRateAdapter<void, AVR::ISR::Timer<1>::CompareA, alarmTimer, dcfDecoder, oscillator>;

struct IrDecoder {
    static inline void init() {
        Irmp::irmp_init();
    }
    static inline void rateProcess() {
        Irmp::irmp_ISR();        
    }
    static inline void start() {
    }
};

using irDecoder = IrDecoder;

using irTimer = AVR::Timer8Bit<2>; // timer 2
using irConstantRate = ConstantRateAdapter<void, AVR::ISR::Timer<2>::CompareA, irDecoder>;

using isrRegistrar = IsrRegistrar<systemConstantRate, terminalDevice::RxHandler, terminalDevice::TxHandler, irConstantRate>;
//using isrRegistrar = IsrRegistrar<systemConstantRate, irConstantRate>;

const auto blinkTimer = alarmTimer::create(100_ms, AlarmFlags::Periodic);
const auto secondsTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);
const auto preStartTimer = alarmTimer::create(240_s, AlarmFlags::OneShot | AlarmFlags::Disabled);

std::chrono::system_clock<> systemClock;

std::percent brightness = 10_ppc;

struct StateManager{
    using State = RadioClock::State;
    static void enter(State state) {
        switch(state) {
        case State::PreStart:
            std::cout << "S: PreStart"_pgm << std::endl;
            powerSwitchPin::off();
            statusLed::steadyColor(Constant::cOff);
            statusLed::enable();
            break;
        case State::Start:
            std::cout << "S: Start"_pgm << std::endl;
            powerSwitchPin::off();
            statusLed::disable();
            statusLed::steadyColor(Constant::cBlue);
            statusLed::update(brightness);
            break;
        case State::Sync:
            std::cout << "S: Sync"_pgm << std::endl;
            powerSwitchPin::off();
            statusLed::disable();
            statusLed::steadyColor(Constant::cRed);
            statusLed::update(brightness);
            break;
        case State::Clock:
            std::cout << "S: Clock"_pgm << std::endl;
            systemClock = std::chrono::system_clock<>::from(dcfDecoder::dateTime());
            EventManager::enqueue(EventType::SystemClockSet);
            powerSwitchPin::on();
            statusLed::enable();
            statusLed::steadyColor(Constant::cGreen);
            break;
        case State::Error:
            std::cout << "S: Error"_pgm << std::endl;
            powerSwitchPin::off();
            statusLed::disable();
            statusLed::steadyColor(Constant::cMagenta);
            statusLed::update(brightness);
            break;
        }
    }        
};

static void flash(bool bit) {
    if (bit) {
        statusLed::flash(Constant::cRed * brightness, 2);
    }
    else {
        statusLed::flash(Constant::cRed * brightness, 1);
    }
}
using radioClock = RadioClock::Clock<dcfDecoder, StateManager, flash>;

template<typename SiftText = void>
struct GlobalStateMachine {
    enum class State : uint8_t {Init, Start, Clock, Date, Temp, Text};
    enum class Event : uint8_t {Start, Clock, StateSwitchFw, StateSwitchBw, FastTick, SecondTick};
    static void process(Event e) {
        State newState = mState;
        switch(mState) {
        case State::Init:
            if (e == Event::Start) {
                newState = State::Start;
            }
            else if (e == Event::FastTick) {
                statusLed::tick(brightness);
            }
            break;
        case State::Start:
            if (e == Event::Clock) {
                newState = State::Clock;
            }
            else if (e == Event::SecondTick) {
            }
            else if (e == Event::FastTick) {
                statusLed::tick(brightness);
            }
            break;
        case State::Clock:
            if (e == Event::StateSwitchFw) {
                newState = State::Date;
            }
            else if (e == Event::StateSwitchBw) {
                newState = State::Text;
            }
            else if (e == Event::Start) {
                newState = State::Start;
            }
            else if (e == Event::SecondTick) {
                if constexpr(useLight) {
                    std::cout << "light: "_pgm << brightness << std::endl;
                    display::brightness(brightness);
                }
                display::set(systemClock);
            }
            else if (e == Event::FastTick) {
                statusLed::tick(brightness);
            }
            break;
        case State::Date:
            if (e == Event::StateSwitchFw) {
                newState = State::Temp;
            }
            else if (e == Event::StateSwitchBw) {
                newState = State::Clock;
            }
            else if (e == Event::Start) {
                newState = State::Start;
            }
            else if (e == Event::SecondTick) {
//                StringBuffer<30> sb;
//                isotime_r(&systemClock.dateTime().tm(), sb.begin());
//                shifttext::set(sb);
//                shifttext::write();
            }
            else if (e == Event::FastTick) {
//                statusLed::tick(brightness);
//                shifttext::shift();
//                shifttext::write();
            }
            break;
            break;
        case State::Temp:
            if (e == Event::StateSwitchFw) {
                newState = State::Text;
            }
            else if (e == Event::StateSwitchBw) {
                newState = State::Date;
            }
            else if (e == Event::Start) {
                newState = State::Start;
            }
            break;
        case State::Text:
            if (e == Event::StateSwitchFw) {
                newState = State::Clock;
            }
            else if (e == Event::StateSwitchBw) {
                newState = State::Temp;
            }
            else if (e == Event::Start) {
                newState = State::Start;
            }
            else if (e == Event::FastTick) {
                statusLed::tick(brightness);
//                shifttext::shift();
//                shifttext::write();
            }
            break;
        }
        if (newState != mState) {
            mState = newState;
            switch(mState) {
            case State::Init:
                std::cout << "G: Init"_pgm << std::endl;
                break;
            case State::Start:
                std::cout << "G: Start"_pgm << std::endl;
                EventManager::enqueue(EventType::RadioClockStart);
                break;
            case State::Clock:
                std::cout << "G: Clock"_pgm << std::endl;
                display::clear();
                break;
            case State::Date:
                display::clear();
//                shifttext::clear();
//                shifttext::reset();
                std::cout << "G: Date"_pgm << std::endl;
                break;
            case State::Temp:
                display::clear();
//                shifttext::clear();
//                shifttext::reset();
                std::cout << "G: Temp"_pgm << std::endl;
                break;
            case State::Text:
                display::clear();
//                shifttext::clear();
//                shifttext::reset();
//                shifttext::set(Constant::title);
                std::cout << "G: Text"_pgm << std::endl;
                break;
            }
        }
    }
private:
    inline static State mState = State::Init;
};

using globalFSM = GlobalStateMachine<shifttext>;

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(std::byte b) {
        auto timer = std::to_integer<uint7_t>(b);
        if (timer == *blinkTimer) {
            globalFSM::process(globalFSM::Event::FastTick);
        }
        else if (timer == *secondsTimer) {
            if (systemClock) {
                if ((systemClock.value() % Constant::secondsPerDay) == (4 * Constant::secondsPerHour)) {
                    systemClock = -1;
                    std::cout << "ReSync"_pgm << std::endl;
                    globalFSM::process(globalFSM::Event::Start);
                }
            }
            systemClock.tick();            
            globalFSM::process(globalFSM::Event::SecondTick);
        }
        else if (timer == *preStartTimer) {
            globalFSM::process(globalFSM::Event::Start);
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

struct IRHandler : public EventHandler<EventType::IREvent> {
    static bool process(std::byte c) {
        std::outl<terminal>("IR: "_pgm, c);
        if (mLastCode != 0_B) {
            if (c == mLastCode) {
                globalFSM::process(globalFSM::Event::StateSwitchFw);
            }
            else {
                globalFSM::process(globalFSM::Event::StateSwitchBw);
            }
        }    
        else {
            globalFSM::process(globalFSM::Event::StateSwitchFw);
            mLastCode = c;
        }
        return true;
    }  
    static inline std::byte mLastCode = 0_B;
    
};
struct IRRepeatHandler : public EventHandler<EventType::IREventRepeat> {
    static bool process(std::byte c) {
        std::outl<terminal>("IR R: "_pgm, c);
        return true;
    }  
};
struct SystemClockSet : public EventHandler<EventType::SystemClockSet> {
    static bool process(std::byte c) {
        std::outl<terminal>("Clock set"_pgm, c);
        globalFSM::process(globalFSM::Event::Clock);
        return true;
    }  
};

using allEventHandler = EventHandlerGroup<TimerHandler, UsartFeHandler, UsartUpeHandler, UsartDorHandler, Usart0Handler,
IRHandler, IRRepeatHandler,
SystemClockSet>;

constexpr std::hertz fIr = 15000_Hz;

int main() {   
    powerSwitchPin::dir<AVR::Output>();    
    powerSwitchPin::off();    
    isrRegistrar::init();
    terminalDevice::init<19200>();
    statusLed::init();
    display::init();
    dcfDecoder::init();
    radioClock::init();
    
    systemTimer::template prescale<LocalConfig::tsd.prescaler>();
    systemTimer::template ocra<LocalConfig::tsd.ocr - 1>();
    systemTimer::mode(AVR::TimerMode::CTC);
    systemTimer::start();
    
    if constexpr(useLight) {
        adc::init();
    }
    if constexpr(useIR) {
        constexpr auto tsd = AVR::Util::calculate<irTimer>(fIr);
        static_assert(tsd, "wrong parameter");
        irTimer::prescale<tsd.prescaler>();
        irTimer::ocra<tsd.ocr>();
        irTimer::mode(AVR::TimerMode::CTC);
        irConstantRate::init();
    }
    
    {
        Scoped<EnableInterrupt> ei;
        std::cout << Constant::title << std::endl;
        
        std::cout << "OSCCAL: "_pgm << oscillator::calibration() << std::endl;
        std::cout << "f systemtimer: "_pgm << LocalConfig::tsd.f << std::endl;
        
        if constexpr(useText) {
            Scoped st{
                [](){powerSwitchPin::on();},
                [](){powerSwitchPin::off();}
            };
//            shifttext::set(Constant::title);
//            for(uint8_t i = 0; i < (Constant::title.size + 1) * shifttext::font().Width; ++i) {
//                shifttext::write();
//                shifttext::shift();
//                Util::delay(300_ms);
//            }
        }
        
        alarmTimer::start(*preStartTimer);
        
        EventManager::run3<allEventHandler, radioClock::HandlerGroup>([](){
            if constexpr(useLight) {
                adc::periodic();
                brightness = std::scale(adc::value(0), adc::value_type(0), adc::value_type(Constant::analogBrightnessMaximum));
                brightness = std::max(brightness, 1_ppc);
            }
            
            systemConstantRate::periodic();
            if constexpr(useIR) {
                irConstantRate::periodic();
                Irmp::IRMP_DATA irmp_data;
                if (irmp_get_data(&irmp_data)) {
                    EventManager::enqueue({(irmp_data.flags & Irmp::Repetition) ? EventType::IREventRepeat : EventType::IREvent , 
                                           std::byte(irmp_data.command)});
                }
            }
            
            if (EventManager::unprocessedEvent()) {
                EventManager::unprocessedEvent() = false;
                statusLed::enable();
                statusLed::flash(Constant::cMagenta, 10);
                std::cout << "upe"_pgm << std::endl;
            }
            if (EventManager::leakedEvent()) {
                EventManager::leakedEvent() = false;
                statusLed::enable();
                statusLed::flash(Constant::cYellow, 10);
                std::cout << "le"_pgm << std::endl;
            }
        });
    }
}

ISR(TIMER1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}
ISR(TIMER2_COMPA_vect) {
    if constexpr(useIR) {
        isrRegistrar::isr<AVR::ISR::Timer<2>::CompareA>();
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

constexpr uint32_t finterrupts = fIr.value;

static_assert(finterrupts >= 10000, "IR Interrupts frequeny too low");
static_assert(finterrupts <= 20000, "IR Interrupts frequeny too high");

#define F_INTERRUPTS finterrupts

// uncomment in original file
#define input(x) iRPin::read()

namespace Irmp {
// this must be the last statement!!!
//#include "irmp/irmp.c"
}
