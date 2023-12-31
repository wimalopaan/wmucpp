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

// 20MHz full-swing
// sudo avrdude -p atmega1284P -P usb -c avrisp2 -U lfuse:w:0xf7:m -U hfuse:w:0xd1:m -U efuse:w:0xfc:m

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
#include "hal/event.h"
#include "hal/constantrate.h"
#include "hal/alarmtimer.h"
#include "hal/adccontroller.h"
#include "external/ws2812.h"
#include "external/dcf77.h"
#include "external/lm35.h"
#include "external/irmp.h"
#include "appl/blink.h"
#include "appl/ledflash.h"
#include "appl/wordclock.h"
#include "appl/fonts.h"
#include "appl/shiftdisplay.h"
#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using adc = AdcController<AVR::Adc<0, AVR::Resolution<10>>, 0, 1>;

using iRPin          = AVR::Pin<PortA, 2>;
using dcfPin         = AVR::Pin<PortA, 3>;
using powerSwitchPin = AVR::Pin<PortA, 4>;
using ledsPin        = AVR::Pin<PortD, 4>;
using testPin        = AVR::Pin<PortD, 5>;
using debugPin       = AVR::Pin<PortD, 7>;

namespace Constant {
static constexpr uint16_t analogBrightnessMaximum = 1023;
static constexpr uint32_t secondsPerDay = (uint32_t)60 * 60 * 24;
static constexpr auto title = "Wordclock 01"_pgm;
static constexpr uint8_t textLength = 64;
}

using display = WordclockDisplay<ledsPin, ColorSequenceGRB>;

using shifttext = ShiftDisplay<display::leds, display::mColumns, display::mRows, Constant::textLength, Font<5,7>>;

using temp = LM35<adc, 1>;

using terminalDevice = AVR::Usart<1, void>;
using terminal = std::basic_ostream<terminalDevice> ;

using systemTimer = AVR::Timer16Bit<1>; // timer 1

struct LocalConfig {
    static constexpr AVR::Util::TimerSetupData tsd = AVR::Util::caculateForExactFrequencyAbove<systemTimer>(Config::Timer::frequency);
    static_assert(tsd, "wrong timer parameter");
    static constexpr std::milliseconds reso = std::duration_cast<std::milliseconds>(1 / tsd.f);
    static constexpr std::hertz exactFrequency = tsd.f;
};

using alarmTimer  = AlarmTimer<systemTimer, UseEvents<true>, LocalConfig::reso>;

using dcfDecoder = DCF77<dcfPin, LocalConfig::exactFrequency, EventManager, true>;

using systemConstantRate = ConstantRateAdapter<1, void, AVR::ISR::Timer<0>::CompareA, alarmTimer, dcfDecoder>;

struct IrDecoder {
    static void init() {
        Irmp::irmp_init();
    }
    static void rateProcess() {
        Irmp::irmp_ISR();        
    }
    static void start() {}
};

using irDecoder = IrDecoder;

using irTimer = AVR::Timer8Bit<2>; // timer 2
using irConstantRate = ConstantRateAdapter<2, void, AVR::ISR::Timer<2>::CompareA, irDecoder>;

using isrRegistrar = IsrRegistrar<systemConstantRate, terminalDevice::RxHandler, terminalDevice::TxHandler, irConstantRate>;

namespace std {
std::basic_ostream<terminalDevice> cout;
std::lineTerminator<CRLF> endl;
}

const auto blinkTimer = alarmTimer::create(100_ms, AlarmFlags::Periodic);
const auto secondsTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);
const auto preStartTimer = alarmTimer::create(3000_ms, AlarmFlags::OneShot | AlarmFlags::Disabled);

std::chrono::system_clock<> systemClock;

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

struct GlobalStateMachine {
    enum class State : uint8_t {Clock, Date, Temp, Text};
    enum class Event : uint8_t {StateSwitch, FastTick, SecondTick};
    static void process(Event) {
        static State state = State::Text;
        State newState = state;
        switch(state) {
        case State::Clock:
            break;
        case State::Date:
            break;
        case State::Temp:
            break;
        case State::Text:
            break;
        }
        if (newState != state) {
            state = newState;
            switch(state) {
            case State::Clock:
                break;
            case State::Date:
                break;
            case State::Temp:
                break;
            case State::Text:
                break;
            }
        }
    }
};

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(std::byte b) {
        auto timer = std::to_integer<uint7_t>(b);
        if (timer == *blinkTimer) {
            debugPin::toggle();
        }
        else if (timer == *secondsTimer) {
            std::cout << "light: "_pgm << display::brightness() << std::endl;
            std::cout << "temp: "_pgm << temp::temperature() << std::endl;
            if (systemClock) {
                std::cout << systemClock.dateTime() << std::endl;
                display::set(systemClock);
                
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
        return true;
    }  
};
struct DCFReceive1Handler : public EventHandler<EventType::DCFReceive1> {
    static bool process(std::byte) {
        return true;
    }  
};
struct DCFDecodeHandler : public EventHandler<EventType::DCFDecode> {
    static bool process(std::byte) {
        ClockStateMachine::process(ClockStateMachine::Event::DCFDecode);
        return true;
    }  
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
struct IRHandler : public EventHandler<EventType::IREvent> {
    static bool process(std::byte c) {
        std::outl<terminal>("IR: "_pgm, c);;
        return true;
    }  
};
struct IRRepeatHandler : public EventHandler<EventType::IREventRepeat> {
    static bool process(std::byte c) {
        std::outl<terminal>("IR R: "_pgm, c);
        return true;
    }  
};

using allEventHandler = EventHandlerGroup<TimerHandler, UsartFeHandler, UsartUpeHandler, UsartDorHandler, Usart0Handler,
                                          DCFReceive0Handler, DCFReceive1Handler, DCFDecodeHandler, DCFSyncHandler, DCFErrorHandler, DCFParityHandler,
                                          IRHandler, IRRepeatHandler>;

constexpr std::hertz fIr = 15000_Hz;

int main() {   
    MCU::Ressource::Registrar<systemConstantRate, irConstantRate>::init();
    isrRegistrar::init();
    
    debugPin::dir<AVR::Output>();
    debugPin::off();
    
    terminalDevice::init<19200>();
    
    systemTimer::template prescale<LocalConfig::tsd.prescaler>();
    systemTimer::template ocra<LocalConfig::tsd.ocr - 1>();
    systemTimer::mode(AVR::TimerMode::CTC);
    systemTimer::start();

    constexpr auto tsd = AVR::Util::calculate<irTimer>(fIr);
    static_assert(tsd, "wrong parameter");
    irTimer::prescale<tsd.prescaler>();
    irTimer::ocra<tsd.ocr>();
    irTimer::mode(AVR::TimerMode::CTC);
    
    irConstantRate::init();
    
    
    powerSwitchPin::dir<AVR::Output>();    
    powerSwitchPin::on();    
    
    Util::delay(500_ms);
    display::init();
    
    StringBuffer<Constant::textLength> sb;
    sb.insertAt(0, Constant::title);
    shifttext::set(sb);
    
    for(uint8_t i = 0; i < 30; ++i) {
        shifttext::write();
        shifttext::shift();
        Util::delay(200_ms);
    }

    display::clear();
    Util::delay(1000_ms);
    
    powerSwitchPin::off();    
    
    testPin::dir<AVR::Output>();
    testPin::low();
    
    adc::init();
    
    {
        Scoped<EnableInterrupt<>> ei;
        std::cout << Constant::title << std::endl;
        alarmTimer::start(*preStartTimer);
        EventManager::run2<allEventHandler>([](){
            adc::periodic();
            // todo: move out
//            display::brightness(std::scale(adc::value(0), adc::value_type(0), adc::value_type(Constant::analogBrightnessMaximum)));

            systemConstantRate::periodic();
            irConstantRate::periodic();
            
            Irmp::IRMP_DATA irmp_data;
            if (irmp_get_data(&irmp_data)) {
                EventManager::enqueue({(irmp_data.flags & Irmp::Repetition) ? EventType::IREventRepeat : EventType::IREvent , std::byte(irmp_data.command)});
            }
            if (EventManager::unprocessedEvent()) {
            }
            if (EventManager::leakedEvent()) {
            }
        });
    }
}

ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}
ISR(TIMER2_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<2>::CompareA>();
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

constexpr uint32_t finterrupts = fIr.value;

static_assert(finterrupts >= 10000, "IR Interrupts frequeny too low");
static_assert(finterrupts <= 20000, "IR Interrupts frequeny too high");

#define F_INTERRUPTS finterrupts

// uncomment in original file
#define input(x) iRPin::read()

namespace Irmp {
// this must be the last statement!!!
#include "irmp/irmp.c"
}

