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

// external Osc (Freq see Makefile)
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xd0:m -U hfuse:w:0xd1:m -U efuse:w:0xff:m

// internal 8 MHz
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

//#define SIMAVR

#include <stdlib.h>
#include <util/eu_dst.h>

#include "config.h"
#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/adc.h"
#include "mcu/avr/clock.h"
#include "hal/event.h"
#include "hal/constantrate.h"
#include "hal/alarmtimer.h"
#include "external/ws2812.h"
#include "external/dcf77.h"
#include "std/chrono.h"
#ifdef SIMAVR
# include "simavr/simavrdebugconsole.h"
#endif
#include "console.h"

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using iRPin    = AVR::Pin<PortB, 0>;
using d0_ss_Pin = AVR::Pin<PortD, 6>;
using d1_data_Pin = AVR::Pin<PortD, 5>;
using displayLedsPin = d1_data_Pin;
using d2_clck_Pin = AVR::Pin<PortB, 3>;
using d3_Pin = AVR::Pin<PortD, 3>;

using rxdPin         = AVR::Pin<PortD, 0>;
using txdPin         = AVR::Pin<PortD, 1>;
using dcfPin         = AVR::Pin<PortC, 2>;
using powerSwitchPin = AVR::Pin<PortD, 2>;

using ledPin         = AVR::Pin<PortB, 7>;
using led = WS2812<1, ledPin, ColorSequenceGRB>;
using Color = led::color_type;

using display = WS2812<10 * 11 + 4, d0_ss_Pin, ColorSequenceGRB>;


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
static constexpr Color cWhite{Red{brightness}, Green{brightness}, Blue{brightness}};
static constexpr Color cWhiteLow{Red{brightness / 10}, Green{brightness / 10}, Blue{brightness / 10}};

static constexpr uint16_t analogBrightnessMaximum = 400;

static constexpr uint32_t secondsPerHour= (uint32_t)60 * 60;
static constexpr uint32_t secondsPerDay = secondsPerHour * 24;

static constexpr auto title = "Test 02"_pgm;
}

#ifdef SIMAVR
using terminal = SimAVRDebugConsole;
#else
using terminal = AVR::Usart<0, void>;
#endif

namespace std {
std::basic_ostream<terminal> cout;
std::lineTerminator<CRLF> endl;
}

using systemTimer = AVR::Timer16Bit<1>;

struct LocalConfig {
    static constexpr AVR::Util::TimerSetupData tsd = AVR::Util::caculateForExactFrequencyAbove<systemTimer>(Config::Timer::frequency);
    static_assert(tsd, "wrong timer parameter");
    static constexpr std::milliseconds reso = std::duration_cast<std::milliseconds>(1 / tsd.f);
    static constexpr std::hertz exactFrequency = tsd.f;
};

using alarmTimer  = AlarmTimer<systemTimer, LocalConfig::reso>;

using dcfDecoder = DCF77<dcfPin, LocalConfig::exactFrequency, EventManager, true>;

using oscillator = AVR::Clock<LocalConfig::exactFrequency, d1_data_Pin>;

using systemConstantRate = ConstantRateAdapter<void, AVR::ISR::Timer<1>::CompareA, alarmTimer, dcfDecoder, oscillator>;

#ifdef SIMAVR
using isrRegistrar = IsrRegistrar<systemConstantRate>;
#else
using isrRegistrar = IsrRegistrar<systemConstantRate, terminal::RxHandler, terminal::TxHandler>;
#endif
const auto secondsTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);

std::chrono::system_clock<> systemClock;

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(uint8_t timer) {
        if (timer == *secondsTimer) {
            systemClock.tick();
            if (systemClock) {
                std::cout << "sc: "_pgm << systemClock.dateTime() << std::endl;
            }
            else {
                std::cout << "tick"_pgm << std::endl;
            }
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
        if (!systemClock) {
            std::cout << "set time"_pgm << std::endl;
            systemClock = std::chrono::system_clock<>::from(dcfDecoder::dateTime());
        }
        if (mCount == 0) {
            oscillator::referenceTime(dcfDecoder::dateTime());
            std::cout << "Delta: "_pgm << oscillator::delta() << std::endl;
            std::cout << "Durat: "_pgm << oscillator::lastMeasurementDuration().value << std::endl;
        }
        mCount = (mCount + 1) % 10;
        return true;
    }  
    inline static uint8_t mCount = 0;
};
struct DCFSyncHandler : public EventHandler<EventType::DCFSync> {
    static bool process(uint8_t) {
        return true;
    }  
};
struct DCFErrorHandler : public EventHandler<EventType::DCFError> {
    static bool process(uint8_t) {
        return true;
    }  
};
struct DCFParityHandler : public EventHandler<EventType::DCFParityError> {
    static bool process(uint8_t) {
        return true;
    }  
};

using allEventHandler = EventHandlerGroup<TimerHandler, UsartFeHandler, UsartUpeHandler, UsartDorHandler, Usart0Handler,
                                        DCFReceive0Handler, DCFReceive1Handler, DCFDecodeHandler, DCFSyncHandler, DCFErrorHandler, DCFParityHandler>;

int main() {   
    powerSwitchPin::dir<AVR::Output>();    
    powerSwitchPin::on();    
    
    display::init();
    
    isrRegistrar::init();
    terminal::init<19200>();
    
    systemTimer::template prescale<LocalConfig::tsd.prescaler>();
    systemTimer::template ocra<LocalConfig::tsd.ocr - 1>();
    systemTimer::mode(AVR::TimerMode::CTC);
    systemTimer::start();
    
    {
        Scoped<EnableInterrupt> ei;
        std::cout << Constant::title << std::endl;
        
        std::cout << "f systemtimer: "_pgm << LocalConfig::tsd.f << std::endl;
        std::cout << "f prescaler: "_pgm << LocalConfig::tsd.prescaler << std::endl;
        std::cout << "f ocr: "_pgm << LocalConfig::tsd.ocr << std::endl;
        std::cout << "f reso: "_pgm << LocalConfig::reso << std::endl;
        std::cout << "a reso: "_pgm << alarmTimer::resolution << std::endl;
    
        while(true) {
            for(uint8_t i = 0; i < display::size; ++i) {
                display::set(i, Constant::cWhiteLow);
                Util::delay(100_ms);
            }
            display::set(Constant::cOff);
        }
        
        
        EventManager::run2<allEventHandler>([](){
            systemConstantRate::periodic();
        });
    }
}

ISR(TIMER1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}
#ifndef SIMAVR
ISR(USART_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}
#endif

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif

