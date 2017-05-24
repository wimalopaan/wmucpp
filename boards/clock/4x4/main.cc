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

// 20 MHz ext.
// sudo avrdude -p atmega88p -P usb -c avrisp2 -U lfuse:w:0xd0:m -U hfuse:w:0xdf:m -U efuse:w:0xf9:m
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xd0:m -U hfuse:w:0xdf:m -U efuse:w:0xf9:m

// 8 MHz int.
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

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

static constexpr bool useStatusLed = true;
static constexpr bool useText = false;
static constexpr bool useUsart = true;
static constexpr bool useLight = true;
static constexpr bool useIR = false;
static constexpr bool useRCCalibration = false;

using d0_ss_Pin = leds2Pin;
using d1_data_Pin = spiDataPin;
using displayLedsPin = leds1Pin;
using d2_clck_Pin = spiDataPin;
//using d3_Pin = AVR::Pin<PortD, 3>;

using display4x4Leds = WS2812<16, displayLedsPin, ColorSequenceGRB>;
using display60Leds = WS2812<60, displayLedsPin, ColorSequenceGRB>;

using led = WS2812<1, ledPin, ColorSequenceGRB>;
using Color = display4x4Leds::color_type;

using spi = SoftSpiMaster<d1_data_Pin, d2_clck_Pin>;

using adc = std::conditional<useLight, AdcController<AVR::Adc<0, AVR::Resolution<8>>, 0>, void>::type;

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

using terminalDevice = std::conditional<useUsart, AVR::Usart<0, void>, void>::type;
using terminal = std::basic_ostream<terminalDevice>;

using statusLed = std::conditional<useStatusLed, LedFlash<led>, void>::type;

using systemTimer = AVR::Timer16Bit<1>;
using irTimer = AVR::Timer8Bit<2>; // timer 2

struct LocalConfig {
    static constexpr AVR::Util::TimerSetupData tsd = AVR::Util::caculateForExactFrequencyAbove<systemTimer>(Config::Timer::frequency);
    static_assert(tsd, "wrong timer parameter");
    static constexpr std::milliseconds reso = std::duration_cast<std::milliseconds>(1 / tsd.f);
    static constexpr std::hertz exactFrequency = tsd.f;
    
    static constexpr AVR::Util::TimerSetupData tsdir = AVR::Util::calculate<irTimer>(10000_Hz);
};

using alarmTimer  = AlarmTimer<systemTimer, LocalConfig::reso>;

using dcfDecoder = DCF77<dcfPin, LocalConfig::exactFrequency, EventManager, true>;

using oscillator = std::conditional<useRCCalibration, AVR::Clock<LocalConfig::exactFrequency>, void>::type;

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
    static inline void check() {
        Irmp::IRMP_DATA irmp_data{};
        if (irmp_get_data(&irmp_data)) {
            EventManager::enqueue({(irmp_data.flags & Irmp::Repetition) ? EventType::IREventRepeat : EventType::IREvent , 
                                   std::byte(irmp_data.command)});
        }
    }
};

using irDecoder = std::conditional<useIR, IrDecoder, void>::type;

using irConstantRate = ConstantRateAdapter<void, AVR::ISR::Timer<2>::CompareA, irDecoder>;

using TerminalRxHandler = AVR::Util::RxHandler<terminalDevice>::type;
using TerminalTxHandler = AVR::Util::TxHandler<terminalDevice>::type;

using isrRegistrar = IsrRegistrar<systemConstantRate, TerminalRxHandler, TerminalTxHandler, irConstantRate>;

const auto blinkTimer = alarmTimer::create(100_ms, AlarmFlags::Periodic);
const auto secondsTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);
const auto preStartTimer = alarmTimer::create(240_s, AlarmFlags::OneShot | AlarmFlags::Disabled);

std::chrono::system_clock<> systemClock;

template<typename Osc>
struct Calibrator {
    inline static constexpr int32_t thresh = 10;
    inline static uint8_t intervall = 0;
    
    static inline void calibrate() {
        if (auto m = Osc::actualMeasurementDuration(dcfDecoder::dateTime()); m && *m >= Osc::mIntervalls[intervall]) {
            Osc::referenceTime(dcfDecoder::dateTime());
            std::out<terminal>("Delta: "_pgm, Osc::delta());
            std::out<terminal>("Durat: "_pgm, Osc::lastMeasurementDuration().value);
            
            if (auto delta = Osc::delta(); delta != 0) {
                if ((abs(delta) >= thresh)) {
                    if (delta > 0) {
                        Osc::adjust(-1);
                    }
                    else {
                        Osc::adjust(1);
                    }
                    std::outl<terminal>("OSCCAL: "_pgm, Osc::calibration());
                }
                else {
                    intervall = std::min((uint8_t)(intervall + 1), (uint8_t)(Osc::mIntervalls.size - 1));
                    std::outl<terminal>("interval: "_pgm, Osc::mIntervalls[intervall].value);;
                }
            }
        }
    }
};
using calibrator = std::conditional<useRCCalibration, Calibrator<oscillator>, void>::type;

struct BrightnessControl {
    inline static std::percent brightness = 10_ppc;
};

using brightness = std::conditional<useLight, BrightnessControl, void>::type;

template<typename StatusLed, typename BrightnessController = void>
struct StateManager{
    static constexpr bool useStatusLed = !std::is_same<StatusLed, void>::value;
    static constexpr bool useBrightness= !std::is_same<BrightnessController, void>::value;
    using State = RadioClock::State;
    static void enter(State state) {
        switch(state) {
        case State::PreStart:
            std::outl<terminal>("S: PreStart"_pgm);
            powerSwitchPin::off();
            if constexpr(useStatusLed) {
                StatusLed::steadyColor(Constant::cOff);
                StatusLed::enable();
            }
            break;
        case State::Start:
            std::outl<terminal>("S: Start"_pgm);
            powerSwitchPin::off();
            if constexpr(useStatusLed) {
                StatusLed::disable();
                StatusLed::steadyColor(Constant::cBlue);
                if constexpr(useBrightness) {
                    StatusLed::update(BrightnessController::brightness);
                }
            }
            break;
        case State::Sync:
            std::outl<terminal>("S: Sync"_pgm);
            powerSwitchPin::off();
            if constexpr(useStatusLed) {
                StatusLed::disable();
                StatusLed::steadyColor(Constant::cRed);
                if constexpr(useBrightness) {
                    StatusLed::update(BrightnessController::brightness);
                }
            }
            break;
        case State::Clock:
            std::outl<terminal>("S: Clock"_pgm);
            systemClock = std::chrono::system_clock<>::from(dcfDecoder::dateTime());
            EventManager::enqueue(EventType::SystemClockSet);
            powerSwitchPin::on();
            if constexpr(useStatusLed) {
                StatusLed::enable();
                StatusLed::steadyColor(Constant::cGreen);
            }
            break;
        case State::Error:
            std::outl<terminal>("S: Error"_pgm);
            powerSwitchPin::off();
            if constexpr(useStatusLed) {
                StatusLed::disable();
                StatusLed::steadyColor(Constant::cMagenta);
                if constexpr(useBrightness) {
                    StatusLed::update(BrightnessController::brightness);
                }
            }
            break;
        }
    }        
};

template<typename Led, typename BC = void>
class Flasher {
public:
    static void flash(bool bit) {
        if constexpr(!std::is_same<Led, void>::value) {
            if constexpr(std::is_same<BC, void>::value) {
                if (bit) {
                    Led::flash(Constant::cRed, 2);
                }
                else {
                    Led::flash(Constant::cRed, 1);
                }
            }
            else {
                if (bit) {
                    Led::flash(Constant::cRed * BC::brightness, 2);
                }
                else {
                    Led::flash(Constant::cRed * BC::brightness, 1);
                }
            }
        }
}
};

using flash = Flasher<statusLed, brightness>;

using radioClock = RadioClock::Clock<dcfDecoder, StateManager<statusLed, BrightnessControl>, flash, calibrator>;

template<typename ShiftText = void, typename StatusLed = void, typename BC = void>
struct GlobalStateMachine {
    static constexpr bool useStatusLed = !std::is_same<StatusLed, void>::value;
    static constexpr bool useShiftText = !std::is_same<ShiftText, void>::value;
    static constexpr bool useBrightness = !std::is_same<BC, void>::value;
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
                if constexpr(useStatusLed) {
                    if constexpr(useBrightness) {
                        StatusLed::tick(BC::brightness);
                    }
                    else {
                        StatusLed::tick(100_ppc);
                    }
                }
            }
            break;
        case State::Start:
            if (e == Event::Clock) {
                newState = State::Clock;
            }
            else if (e == Event::SecondTick) {
            }
            else if (e == Event::FastTick) {
                if constexpr(useStatusLed) {
                    if constexpr(useBrightness) {
                        StatusLed::tick(BC::brightness);
                    }
                    else {
                        StatusLed::tick(100_ppc);
                    }
                }
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
                if constexpr(useBrightness) {
                    //                    std::outl<terminal>("light: "_pgm, BC::brightness);
                    display::brightness(BC::brightness);
                }
                display::set(systemClock);
            }
            else if (e == Event::FastTick) {
                if constexpr(useStatusLed) {
                    if constexpr(useBrightness) {
                        StatusLed::tick(BC::brightness);
                    }
                    else {
                        StatusLed::tick(100_ppc);
                    }
                }
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
                if constexpr(useShiftText) {
                    StringBuffer<30> sb;
                    isotime_r(&systemClock.dateTime().tm(), sb.begin());
                    ShiftText::set(sb);
                    ShiftText::write();
                }
            }
            else if (e == Event::FastTick) {
                if constexpr(useStatusLed) {
                    if constexpr(useBrightness) {
                        StatusLed::tick(BC::brightness);
                    }
                    else {
                        StatusLed::tick(100_ppc);
                    }
                }
                if constexpr(useShiftText) {
                    ShiftText::shift();
                    ShiftText::write();
                }
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
                if constexpr(useStatusLed) {
                    if constexpr(useBrightness) {
                        StatusLed::tick(BC::brightness);
                    }
                    else {
                        StatusLed::tick(100_ppc);
                    }
                }
                //                shifttext::shift();
                //                shifttext::write();
            }
            break;
        }
        if (newState != mState) {
            mState = newState;
            switch(mState) {
            case State::Init:
                std::outl<terminal>("G: Init"_pgm);
                break;
            case State::Start:
                std::outl<terminal>("G: Start"_pgm);
                EventManager::enqueue(EventType::RadioClockStart);
                break;
            case State::Clock:
                std::outl<terminal>("G: Clock"_pgm);
                display::clear();
                break;
            case State::Date:
                display::clear();
                //                shifttext::clear();
                //                shifttext::reset();
                std::outl<terminal>("G: Date"_pgm);
                break;
            case State::Temp:
                display::clear();
                //                shifttext::clear();
                //                shifttext::reset();
                std::outl<terminal>("G: Temp"_pgm);
                break;
            case State::Text:
                display::clear();
                //                shifttext::clear();
                //                shifttext::reset();
                //                shifttext::set(Constant::title);
                std::outl<terminal>("G: Text"_pgm);
                break;
            }
        }
    }
private:
    inline static State mState = State::Init;
};

using globalFSM = GlobalStateMachine<shifttext, statusLed, brightness>;

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(std::byte b) {
        auto timer = std::to_integer<uint7_t>(b);
        if (timer == *blinkTimer) {
            globalFSM::process(globalFSM::Event::FastTick);
        }
        else if (timer == *secondsTimer) {
            if (systemClock) {
                if ((systemClock.value() % Constant::secondsPerDay) == (13 * Constant::secondsPerHour)) {
                    systemClock = -1;
                    std::outl<terminal>("ReSync"_pgm);
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

using allEventHandler = EventHandlerGroup<TimerHandler, 
UsartFeHandler, UsartUpeHandler, UsartDorHandler, Usart0Handler, 
IRHandler, IRRepeatHandler,
SystemClockSet>;

namespace detail {
    
    template<typename IrDec, typename Adc, typename BC, typename StatusLed, typename ShiftText, typename TerminalDevice, typename RCCalibration>
    void main() {
        powerSwitchPin::dir<AVR::Output>();    
        powerSwitchPin::off();    
        isrRegistrar::init();
        if constexpr (!std::is_same<TerminalDevice, void>::value) {
            TerminalDevice::template init<19200>();
        }
        if constexpr(!std::is_same<StatusLed, void>::value) {
            StatusLed::init();
        }
        
        display::init();
        dcfDecoder::init();
        radioClock::init();
        leds2Pin::dir<AVR::Output>();
        leds2Pin::on();
        
        systemTimer::template prescale<LocalConfig::tsd.prescaler>();
        systemTimer::template ocra<LocalConfig::tsd.ocr - 1>();
        systemTimer::mode(AVR::TimerMode::CTC);
        systemTimer::start();
        
        if constexpr(!std::is_same<Adc, void>::value) {
            Adc::init();
        }
        if constexpr(!std::is_same<IrDec, void>::value) {
            iRPin::dir<AVR::Input>();
            IrDec::init();
            static_assert(LocalConfig::tsdir, "wrong parameter");
            irTimer::prescale<LocalConfig::tsdir.prescaler>();
            irTimer::ocra<LocalConfig::tsdir.ocr>();
            irTimer::mode(AVR::TimerMode::CTC);
            irConstantRate::init();
            std::outl<terminal>("fir: "_pgm, LocalConfig::tsdir.f);
            std::outl<terminal>("ps: "_pgm, LocalConfig::tsdir.prescaler);
            std::outl<terminal>("oc: "_pgm, LocalConfig::tsdir.ocr);
        }
        
        {
            Scoped<EnableInterrupt> ei;
            std::outl<terminal>(Constant::title);
            
            if constexpr(!std::is_same<RCCalibration, void>::value) {
                std::outl<terminal>("OSCCAL: "_pgm, RCCalibration::calibration());
                std::outl<terminal>("f systemtimer: "_pgm, LocalConfig::tsd.f);
            }
            
            if constexpr(!std::is_same<ShiftText, void>::value) {
                Scoped st{
                    [](){powerSwitchPin::on();},
                    [](){powerSwitchPin::off();}
                };
                ShiftText::set(Constant::title);
                for(uint8_t i = 0; i < (Constant::title.size + 1) * ShiftText::font().Width; ++i) {
                    ShiftText::write();
                    ShiftText::shift();
                    Util::delay(300_ms);
                }
            }
            {
                std::outl<terminal>("Test..."_pgm);
                Scoped st{
                    [](){powerSwitchPin::on();},
                    [](){powerSwitchPin::off();}
                };
                display4x4Leds::off();
                for(uint8_t i = 0 ; i < display4x4Leds::size; ++i) {
                    display4x4Leds::set(i, Constant::cGreen);
                    Util::delay(100_ms);
                }
                std::outl<terminal>("...End"_pgm);
            }
            
            alarmTimer::start(*preStartTimer);
            
            EventManager::run3<allEventHandler, radioClock::HandlerGroup>([](){
//                leds2Pin::toggle();
                if constexpr(!std::is_same<BC, void>::value && !std::is_same<Adc, void>::value) {
                    Adc::periodic();
                    BC::brightness = std::fastScale(Adc::value(0));
                    BC::brightness = std::max(BC::brightness, 1_ppc);
                }
                systemConstantRate::periodic();
                if constexpr(!std::is_same<IrDec, void>::value) {
                    irConstantRate::periodic();
                    IrDec::check();
                }
                if (EventManager::unprocessedEvent()) {
                    EventManager::unprocessedEvent() = false;
                    if constexpr(!std::is_same<StatusLed, void>::value) {
                        StatusLed::enable();
                        StatusLed::flash(Constant::cMagenta, 10);
                    }                
                    std::outl<terminal>("upe"_pgm);
                }
                if (EventManager::leakedEvent()) {
                    EventManager::leakedEvent() = false;
                    if constexpr(!std::is_same<StatusLed, void>::value) {
                        StatusLed::enable();
                        StatusLed::flash(Constant::cYellow, 10);
                    }
                    std::outl<terminal>("le"_pgm);
                }
            });
        }
    }
} //!detail

int main() {   
    detail::main<irDecoder, adc, brightness, statusLed, shifttext, terminalDevice, oscillator>();
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
    if constexpr(useUsart) {
        isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
    }
    
}
ISR(USART_UDRE_vect){
    if constexpr(useUsart) {
        isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
    }
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, ',', file, ',', line);
    while(true) {}
}
#endif

constexpr uint32_t finterrupts = LocalConfig::tsdir.f.value;

static_assert(finterrupts >= 10000, "IR Interrupts frequeny too low");
static_assert(finterrupts <= 20000, "IR Interrupts frequeny too high");

#define F_INTERRUPTS finterrupts

// uncomment in original file
#define input(x) iRPin::read()

namespace Irmp {
    // this must be the last statement!!!
#include "irmp/irmp.c"
}
