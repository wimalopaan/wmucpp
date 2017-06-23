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

// sudo avrdude -p attiny85 -P usb -c avrisp2 -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m
// avrdude -p m328p -c arduino -b 57600 -P /dev/ttyUSB0 -U flash:w:ledbuttons.hex 

#define NDEBUG

#include <stdlib.h>

#include "../../include/anano.h"

#include "mcu/avr/clock.h"
#include "hal/event.h"
#include "hal/constantrate.h"
#include "hal/alarmtimer.h"
#include "mcu/avr/swusart.h"
#include "std/chrono.h"
#include "appl/ledflash.h"
#include "appl/clockstatemachine.h"
#include "appl/blink.h"
#include "appl/twostateblinker.h"
#include "appl/radioclock.h"
#include "console.h"

using led = WS2812<1, ledPin, ColorSequenceGRB>;
using Color = led::color_type;

namespace Constant {
    static constexpr uint8_t brightness = 8;
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
    
    static constexpr auto title = "SW = test; HW = OneLed01"_pgm;
} // !Constant

using blinker = TwoStateBlinker<led>;

using systemTimer = AVR::Timer8Bit<1>;

struct LocalConfig {
    static constexpr AVR::Util::TimerSetupData tsd = AVR::Util::caculateForExactFrequencyAbove<systemTimer>(Config::Timer::frequency);
    static_assert(tsd, "wrong timer parameter");
    static constexpr std::milliseconds reso = std::duration_cast<std::milliseconds>(1 / tsd.f);
    static constexpr std::hertz exactFrequency = tsd.f;
};

using alarmTimer  = AlarmTimer<systemTimer, LocalConfig::reso>;

using systemConstantRate = ConstantRateAdapter<1, void, AVR::ISR::Timer<1>::CompareA, alarmTimer>;

const auto secondsTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);

using terminalDevice = SWUsart<0>;
using terminal = std::basic_ostream<terminalDevice>;

using isrRegistrar = IsrRegistrar<terminalDevice::TxHandler, systemConstantRate>;

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(std::byte b) {
        auto timer = std::to_integer<uint7_t>(b);
        if (timer == *secondsTimer) {
            blinker::tick();
            std::outl<terminal>("tick"_pgm);
        }
        return true;
    }
};

using allEventHandler = EventHandlerGroup<TimerHandler>;

int main() {
    isrRegistrar::init();
    blinker::init();
    blinker::onColor(Constant::cGreen);
    
    terminalDevice::init<19200>();
    btStatus::dir<AVR::Output>();
    dcfPin::dir<AVR::Input>();
    dcfPin::pullup();

    systemTimer::template prescale<LocalConfig::tsd.prescaler>();
    systemTimer::template ocra<LocalConfig::tsd.ocr - 1>();
    systemTimer::mode(AVR::TimerMode::CTC);
    systemTimer::start();
    
    {
        Scoped<EnableInterrupt<>> ei;
        btStatus::toggle();
        std::outl<terminal>(Constant::title);
        EventManager::run3<allEventHandler>([](){
            btStatus::toggle();
            systemConstantRate::periodic();
        });
    }
}

ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}

ISR(TIMER1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}
