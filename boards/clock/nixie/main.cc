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
//#include <util/eu_dst.h>

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/adc.h"
#include "hal/event.h"
#include "hal/constantrate.h"
#include "hal/alarmtimer.h"
//#include "hal/adccontroller.h"
#include "hal/softspimaster.h"
#include "external/ws2812.h"
#include "external/dcf77.h"
#include "external/irmp.h"
#include "appl/ledflash.h"
#include "std/chrono.h"

#include "console.h"

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using iRPin    = AVR::Pin<PortB, 0>;
using spiSSPin = AVR::Pin<PortB, 1>;

//using adc = AdcController<AVR::Adc<0>, 0>;

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

template<typename Spi>
class TimeBCDSerial {
public:
    static void init() {
        Spi::init();
        for(const auto& b: data()) {
            Spi::put(b);
        }
    }

    template<typename Clock>
    static void set(const Clock& clock) {
        DateTime::TimeTm t = clock.dateTime();
        auto hour   = t.hours().value;
        auto minute = t.minutes().value;
        auto seconds = t.seconds().value;
        
        data()[2] = ((hour % 10) << 4) + hour / 10;
        data()[1] = ((minute % 10) << 4) + minute / 10;
        data()[0] = ((seconds % 10) << 4) + seconds / 10;
        
        for(const auto& b: data()) {
            Spi::put(b);
        }
    }    
private:
    static auto& data() {
        static std::array<uint8_t, 3> mData;
        return mData;
    }
};

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

static constexpr uint32_t secondsPerDay = (uint32_t)60 * 60 * 24;

static constexpr auto title = "NixieClock 01"_pgm;
}

using statusLed = LedFlash<led>;

using terminal = AVR::Usart<0, void>;

using systemTimer = AVR::Timer8Bit<0>; // timer 0
using alarmTimer  = AlarmTimer<systemTimer>;

using dcfDecoder = DCF77<dcfPin, Config::Timer::frequency, EventManager, true>;

using systemConstantRate = ConstantRateAdapter<void, AVR::ISR::Timer<0>::CompareA, alarmTimer, dcfDecoder>;

struct IrDecoder {
    static void init() {
        Irmp::irmp_init();
    }
    static void rateProcess() {
        Irmp::irmp_ISR();        
    }
};

using irDecoder = IrDecoder;

using irTimer = AVR::Timer8Bit<2>; // timer 2
using irConstantRate = ConstantRateAdapter<void, AVR::ISR::Timer<2>::CompareA, irDecoder>;

using isrRegistrar = IsrRegistrar<systemConstantRate, terminal::RxHandler, terminal::TxHandler, irConstantRate>;

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
template<typename PowerPin = void>
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
                if constexpr(!std::is_same<PowerPin, void>::value) {
                    powerSwitchPin::off();
                }
                break;
            case State::Start:
                std::cout << "S: Start"_pgm << std::endl;
                statusLed::steadyColor(Constant::cBlue * brightness);
                if constexpr(!std::is_same<PowerPin, void>::value) {
                    powerSwitchPin::off();
                }
                break;
            case State::Sync1:
                std::cout << "S: Sync1"_pgm << std::endl;
                statusLed::steadyColor(Constant::cYellow * brightness);
                if constexpr(!std::is_same<PowerPin, void>::value) {
                    powerSwitchPin::off();
                }
                break;
            case State::Sync2:
                std::cout << "S: Sync2"_pgm << std::endl;
                statusLed::steadyColor(Constant::cCyan * brightness);
                if constexpr(!std::is_same<PowerPin, void>::value) {
                    powerSwitchPin::off();
                }
                break;
            case State::Clock:
                std::cout << "S: Clock"_pgm << std::endl;
                statusLed::steadyColor(Constant::cGreen * brightness);
                systemClock = std::chrono::system_clock<>::from(dcfDecoder::dateTime());
                if constexpr(!std::is_same<PowerPin, void>::value) {
                    powerSwitchPin::on();
                }
                break;
            case State::Error:
                std::cout << "S: Error"_pgm << std::endl;
                statusLed::steadyColor(Constant::cMagenta * brightness);
                if constexpr(!std::is_same<PowerPin, void>::value) {
                    powerSwitchPin::off();
                }
                break;
            }
        }
    }        
};

using clockFSM = ClockStateMachine<powerSwitchPin>;

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(uint8_t timer) {
        if (timer == *blinkTimer) {
            statusLed::tick(brightness);
        }
        else if (timer == *secondsTimer) {
            std::cout << "light: "_pgm << brightness << std::endl;
            if (systemClock) {
                display::set(systemClock);

                std::cout << systemClock.dateTime() << std::endl;
                
                if ((systemClock.value() % Constant::secondsPerDay) == 0) {
                    systemClock = -1;
                    clockFSM::process(clockFSM::Event::ReSync);
                    std::cout << "ReSync"_pgm << std::endl;
                }
            }
            systemClock.tick();            
        }
        else if (timer == *preStartTimer) {
            clockFSM::process(clockFSM::Event::Start);
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
        clockFSM::process(clockFSM::Event::DCFDecode);
        return true;
    }  
};
struct DCFSyncHandler : public EventHandler<EventType::DCFSync> {
    static bool process(uint8_t) {
        clockFSM::process(clockFSM::Event::DCFSync);
        return true;
    }  
};
struct DCFErrorHandler : public EventHandler<EventType::DCFError> {
    static bool process(uint8_t) {
        clockFSM::process(clockFSM::Event::DCFError);
        return true;
    }  
};
struct DCFParityHandler : public EventHandler<EventType::DCFParityError> {
    static bool process(uint8_t) {
        clockFSM::process(clockFSM::Event::DCFError);
        return true;
    }  
};
struct IRHandler : public EventHandler<EventType::IREvent> {
    static bool process(uint8_t c) {
        std::cout << "IR: "_pgm << c << std::endl;
        return true;
    }  
};
struct IRRepeatHandler : public EventHandler<EventType::IREventRepeat> {
    static bool process(uint8_t c) {
        std::cout << "IR R: "_pgm << c << std::endl;
        return true;
    }  
};

using allEventHandler = EventHandlerGroup<TimerHandler, UsartFeHandler, UsartUpeHandler, UsartDorHandler, Usart0Handler,
                                        DCFReceive0Handler, DCFReceive1Handler, DCFDecodeHandler, DCFSyncHandler, DCFErrorHandler, DCFParityHandler,
                                        IRHandler, IRRepeatHandler
>;

constexpr std::hertz fIr = 15000_Hz;

int main() {   
    // not neccessary because of dcf77 dst flag
//    set_zone(ONE_HOUR); // europe central time
//    set_dst(eu_dst);
    
    isrRegistrar::init();
    alarmTimer::init();
    
    powerSwitchPin::dir<AVR::Output>();    
    powerSwitchPin::off();    
    
    terminal::init<19200>();
    
    statusLed::init();
    
    display::init();
    
//    adc::init();
    
    {
        Scoped<EnableInterrupt> ei;
        std::cout << Constant::title << std::endl;
        alarmTimer::start(*preStartTimer);
        EventManager::run2<allEventHandler>([](){
//            adc::periodic();
//            brightness = std::scale(adc::value(0), adc::value_type(0), adc::value_type(Constant::analogBrightnessMaximum));

            systemConstantRate::periodic();
            irConstantRate::periodic();
            
            Irmp::IRMP_DATA irmp_data;
            if (irmp_get_data(&irmp_data)) {
                EventManager::enqueue({(irmp_data.flags & Irmp::Repetition) ? EventType::IREventRepeat : EventType::IREvent , uint8_t(irmp_data.command)});
            }
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
ISR(TIMER2_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<2>::CompareA>();
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
#include "irmp/irmp.c"
}

