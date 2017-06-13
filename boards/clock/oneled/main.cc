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

#define NDEBUG

#include <stdlib.h>

#include "../../include/oneled01.h"

#include "mcu/avr/clock.h"
#include "mcu/avr/swusart.h"
#include "hal/event.h"
#include "hal/constantrate.h"
#include "hal/alarmtimer.h"
#include "hal/softspimaster.h"
#include "std/chrono.h"
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

static constexpr bool useStatusLed = false;
static constexpr bool useText = false;
static constexpr bool useUsart = false;
static constexpr bool useLight = false;
static constexpr bool useIR = false;
static constexpr bool useRCCalibration = false;

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
    
    static constexpr uint32_t secondsPerHour= (uint32_t)60 * 60;
    static constexpr uint32_t secondsPerDay = secondsPerHour * 24;
    
    static constexpr auto title = "SW = Wastetimer; HW = OneLed"_pgm;
    
} // !Constant

using terminalDevice = SWUsart<0>;
using terminal = std::basic_ostream<terminalDevice>;

using systemTimer = AVR::Timer8Bit<1>;

struct LocalConfig {
    static constexpr AVR::Util::TimerSetupData tsd = AVR::Util::caculateForExactFrequencyAbove<systemTimer>(Config::Timer::frequency);
    static_assert(tsd, "wrong timer parameter");
    static constexpr std::milliseconds reso = std::duration_cast<std::milliseconds>(1 / tsd.f);
    static constexpr std::hertz exactFrequency = tsd.f;
};

using alarmTimer  = AlarmTimer<systemTimer, LocalConfig::reso>;

using dcfDecoder = DCF77<dcfPin, LocalConfig::exactFrequency, EventManager, true>;

using oscillator = std::conditional<useRCCalibration, AVR::Clock<LocalConfig::exactFrequency>, void>::type;

using systemConstantRate = ConstantRateAdapter<1, void, AVR::ISR::Timer<1>::CompareA, alarmTimer, dcfDecoder, oscillator>;

using isrRegistrar = IsrRegistrar<systemConstantRate, terminalDevice::TransmitBitHandler>;

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
            break;
        case State::Start:
            std::outl<terminal>("S: Start"_pgm);
            break;
        case State::Sync:
            std::outl<terminal>("S: Sync"_pgm);
            break;
        case State::Clock:
            std::outl<terminal>("S: Clock"_pgm);
            systemClock = std::chrono::system_clock<>::from(dcfDecoder::dateTime());
            EventManager::enqueue(EventType::SystemClockSet);
            break;
        case State::Error:
            std::outl<terminal>("S: Error"_pgm);
            break;
        }
    }        
};

using radioClock = RadioClock::Clock<dcfDecoder, StateManager<void, BrightnessControl>, void, calibrator>;

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
            }
            break;
        case State::Start:
            if (e == Event::Clock) {
                newState = State::Clock;
            }
            else if (e == Event::SecondTick) {
            }
            else if (e == Event::FastTick) {
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
            }
            else if (e == Event::FastTick) {
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
            }
            else if (e == Event::FastTick) {
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
                    break;
                case State::Date:
                    std::outl<terminal>("G: Date"_pgm);
                    break;
                case State::Temp:
                    std::outl<terminal>("G: Temp"_pgm);
                    break;
                case State::Text:
                    std::outl<terminal>("G: Text"_pgm);
                    break;
                }
            }
        }
    }
private:
    inline static State mState = State::Init;
};

using globalFSM = GlobalStateMachine<void, void, brightness>;

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

struct SystemClockSet : public EventHandler<EventType::SystemClockSet> {
    static bool process(std::byte c) {
        std::outl<terminal>("Clock set"_pgm, c);
        globalFSM::process(globalFSM::Event::Clock);
        return true;
    }  
};

using allEventHandler = EventHandlerGroup<TimerHandler, 
SystemClockSet>;

namespace detail {
    template<typename IrDec, typename Adc, typename BC, typename StatusLed, typename ShiftText, typename TerminalDevice, typename RCCalibration>
    void main() {
        isrRegistrar::init();
        if constexpr (!std::is_same<TerminalDevice, void>::value) {
            TerminalDevice::template init<19200>();
        }
        if constexpr(!std::is_same<StatusLed, void>::value) {
            StatusLed::init();
        }
        
//        display::init();
        dcfDecoder::init();
        radioClock::init();
        
        systemTimer::template prescale<LocalConfig::tsd.prescaler>();
        systemTimer::template ocra<LocalConfig::tsd.ocr - 1>();
        systemTimer::mode(AVR::TimerMode::CTC);
        systemTimer::start();
        
        if constexpr(!std::is_same<Adc, void>::value) {
            Adc::init();
        }
        {
            Scoped<EnableInterrupt<>> ei;
            std::outl<terminal>(Constant::title);
            
            if constexpr(!std::is_same<RCCalibration, void>::value) {
                std::outl<terminal>("OSCCAL: "_pgm, RCCalibration::calibration());
                std::outl<terminal>("f systemtimer: "_pgm, LocalConfig::tsd.f);
            }
            
            alarmTimer::start(*preStartTimer);
            
            EventManager::run3<allEventHandler, radioClock::HandlerGroup>([](){
                systemConstantRate::periodic();
                if (EventManager::unprocessedEvent()) {
                    std::outl<terminal>("upe"_pgm);
                }
                if (EventManager::leakedEvent()) {
                    std::outl<terminal>("le"_pgm);
                }
            });
        }
    }
} //!detail

int main() {   
    detail::main<void, void, brightness, void, void, terminalDevice, oscillator>();
}

ISR(TIMER1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, ',', file, ',', line);
    while(true) {}
}
#endif

