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

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/adc.h"
#include "hal/event.h"
#include "hal/constantrate.h"
#include "hal/alarmtimer.h"
#include "hal/adccontroller.h"
#include "hal/softspimaster.h"
#include "external/ws2812.h"
#include "external/dcf77.h"
#include "std/chrono.h"
#include "appl/ledflash.h"
#include "appl/clockstatemachine.h"
#include "appl/timebcdserial.h"

#include "console.h"

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using iRPin    = AVR::Pin<PortB, 0>;
using spiSSPin = AVR::Pin<PortB, 1>;

using adc = AdcController<AVR::Adc<0, AVR::Resolution<10>>, 0>;

using rxdPin         = AVR::Pin<PortD, 0>;
using txdPin         = AVR::Pin<PortD, 1>;
using dcfPin         = AVR::Pin<PortD, 2>;
using powerSwitchPin = AVR::Pin<PortD, 4>;
using spiClockPin    = AVR::Pin<PortD, 5>;
using spiDataPin     = AVR::Pin<PortD, 6>;
using ledPin         = AVR::Pin<PortD, 7>;

using led = WS2812<1, ledPin, ColorSequenceGRB>;
using Color = led::color_type;

using spi = SoftSpiMaster<spiDataPin, spiClockPin, spiSSPin>;

using display = TimeBCDSerial<spi>;

// todo: belegt RAM im data segment ?
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

static constexpr uint32_t secondsPerHour= (uint32_t)60 * 60;
static constexpr uint32_t secondsPerDay = secondsPerHour * 24;

static constexpr auto title = "NixieClock 02"_pgm;
}

using statusLed = LedFlash<led>;

using terminalDevice = AVR::Usart<0, void>;
using terminal = std::basic_ostream<terminalDevice>;

using systemTimer = AVR::Timer16Bit<1>; // timer 1

struct LocalConfig {
    static constexpr AVR::Util::TimerSetupData tsd = AVR::Util::caculateForExactFrequencyAbove<systemTimer>(Config::Timer::frequency);
    static_assert(tsd, "wrong timer parameter");
    static constexpr std::milliseconds reso = std::duration_cast<std::milliseconds>(1 / tsd.f);
    static constexpr std::hertz exactFrequency = tsd.f;
};

using alarmTimer  = AlarmTimer<systemTimer, LocalConfig::reso>;

using dcfDecoder = DCF77<dcfPin, LocalConfig::exactFrequency, EventManager, true>;

using systemConstantRate = ConstantRateAdapter<1, void, AVR::ISR::Timer<1>::CompareA, alarmTimer, dcfDecoder>;

using isrRegistrar = IsrRegistrar<systemConstantRate, terminalDevice::RxHandler, terminalDevice::TxHandler>;

namespace std {
std::basic_ostream<terminalDevice> cout;
std::lineTerminator<CRLF> endl;
}

const auto blinkTimer = alarmTimer::create(100_ms, AlarmFlags::Periodic);
const auto secondsTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);
const auto preStartTimer = alarmTimer::create(3000_ms, AlarmFlags::OneShot | AlarmFlags::Disabled);

std::chrono::system_clock<> systemClock;

std::percent brightness = 100_ppc;

// todo: für ReSync erweitern: falls Resync nicht erfolgreich -> Zeit beibehalten und ReSyncCounter irgendwie anzeigen (Farbe ander 12).
template<typename PowerPin = void>
struct StateManager{
    using State = typename ClockStateMachine::State;
    static void enter(State state) {
        switch(state) {
        case State::PreStart:
            std::cout << "S: PreStart"_pgm << std::endl;
            if constexpr(!std::is_same<PowerPin, void>::value) {
                powerSwitchPin::off();
            }
            break;
        case State::Start:
            std::cout << "S: Start"_pgm << std::endl;
            if constexpr(!std::is_same<PowerPin, void>::value) {
                powerSwitchPin::off();
            }
            break;
        case State::Sync1:
            std::cout << "S: Sync1"_pgm << std::endl;
            if constexpr(!std::is_same<PowerPin, void>::value) {
                powerSwitchPin::off();
            }
            break;
        case State::Sync2:
            std::cout << "S: Sync2"_pgm << std::endl;
            if constexpr(!std::is_same<PowerPin, void>::value) {
                powerSwitchPin::off();
            }
            break;
        case State::Sync3:
            std::cout << "S: Sync3"_pgm << std::endl;
            if constexpr(!std::is_same<PowerPin, void>::value) {
                powerSwitchPin::off();
            }
            break;
        case State::Clock:
            std::cout << "S: Clock"_pgm << std::endl;
            systemClock = std::chrono::system_clock<>::from(dcfDecoder::dateTime());
            if constexpr(!std::is_same<PowerPin, void>::value) {
                powerSwitchPin::on();
            }
            break;
        case State::Error:
            std::cout << "S: Error"_pgm << std::endl;
            if constexpr(!std::is_same<PowerPin, void>::value) {
                powerSwitchPin::off();
            }
            break;
        }
    }        
};

struct LightManager{
    using State = typename ClockStateMachine::State;
    static void process(State state) {
        switch(state) {
        case State::PreStart:
            statusLed::steadyColor(Constant::cOff);
            break;
        case State::Start:
            statusLed::steadyColor(Constant::cBlue);
            break;
        case State::Sync1:
            statusLed::steadyColor(Constant::cYellow);
            break;
        case State::Sync2:
            statusLed::steadyColor(Constant::cCyan);
            break;
        case State::Sync3:
            statusLed::steadyColor(Constant::cRed);
            break;
        case State::Clock:
            statusLed::steadyColor(Constant::cGreen);
            break;
        case State::Error:
            statusLed::steadyColor(Constant::cMagenta);
            break;
        }
    }        
};

using clockFSM = ClockStateMachine::Machine<StateManager<powerSwitchPin>, void, LightManager>;

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(std::byte b) {
        auto timer = std::to_integer<uint7_t>(b);
        if (timer == *blinkTimer) {
            statusLed::tick(brightness);
        }
        else if (timer == *secondsTimer) {
            std::cout << "light: "_pgm << brightness << std::endl;
            if (systemClock) {
                if ((systemClock.value() % 30) == 0) {
                    display::set(systemClock, TimeDisplay::Mode::Date);
                } 
                else {
                    display::set(systemClock);
                }

                std::cout << systemClock.dateTime() << std::endl;
                
                if ((systemClock.value() % Constant::secondsPerDay) == (4 * Constant::secondsPerHour)) {
                    systemClock = -1;
                    clockFSM::process(ClockStateMachine::Event::ReSync);
                    std::cout << "ReSync"_pgm << std::endl;
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
        if (clockFSM::state() != ClockStateMachine::State::Clock) {
            statusLed::flash(Constant::cRed, 1);
        }
        return true;
    }  
};
struct DCFReceive1Handler : public EventHandler<EventType::DCFReceive1> {
    static bool process(std::byte) {
        if (clockFSM::state() != ClockStateMachine::State::Clock) {
            statusLed::flash(Constant::cRed, 2);
        }
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
struct IRHandler : public EventHandler<EventType::IREvent> {
    static bool process(std::byte c) {
        std::out<terminal>("IR: "_pgm, c);
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
                                        DCFReceive0Handler, DCFReceive1Handler, DCFDecodeHandler, DCFSyncHandler, DCFErrorHandler, DCFParityHandler>;

constexpr std::hertz fIr = 15000_Hz;

int main() {   
    powerSwitchPin::dir<AVR::Output>();    
    powerSwitchPin::off();    
    isrRegistrar::init();
    
    systemTimer::template prescale<LocalConfig::tsd.prescaler>();
    systemTimer::template ocra<LocalConfig::tsd.ocr - 1>();
    systemTimer::mode(AVR::TimerMode::CTC);
    systemTimer::start();
    
    terminalDevice::init<19200>();
    statusLed::init();
    display::init();
    adc::init();
    
    {
        Scoped<EnableInterrupt<>> ei;
        std::cout << Constant::title << std::endl;
        alarmTimer::start(*preStartTimer);
        EventManager::run2<allEventHandler>([](){
            adc::periodic();
            // todo: move out
//            brightness = std::scale(adc::value(0), adc::value_type(0), adc::value_type(Constant::analogBrightnessMaximum));
//            brightness = std::min(brightness, 1_ppc);
            
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
