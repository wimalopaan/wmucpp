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

#include "mcu/avr8.h"
#include "mcu/avr/mcutimer.h"

#include "hal/alarmtimer.h"
#include "hal/event.h"

#include "simavr/simavrdebugconsole.h"

#include "console.h"

namespace  {
    constexpr bool useTerminal = false;
}

struct Test {
    static void periodic() {
        ++x;
    }
    inline static volatile uint8_t x = 0; 
};

using systemTimer = AVR::Timer8Bit<0>;

struct LocalConfig {
    static constexpr AVR::Util::TimerSetupData tsd = AVR::Util::caculateForExactFrequencyAbove<systemTimer>(1000_Hz);
    static_assert(tsd, "wrong timer parameter");
    static constexpr std::milliseconds reso = std::duration_cast<std::milliseconds>(1 / tsd.f);
    static constexpr std::hertz exactFrequency = tsd.f;
};

using terminalDevice = SimAVRDebugConsole;
using terminal = std::conditional<useTerminal, std::basic_ostream<terminalDevice>, void>::type;

using sampler = PeriodicGroup<0, AVR::ISR::Timer<0>::CompareA, Test>;

using isrReg = IsrRegistrar<sampler>; 

// IsrRegistrar<PG<Interrupt1, Callbacks>, CR<Timer, Interrupz2, Callbacks>, otherHandler, ...>

using allEventHandler = EventHandlerGroup<>; 

namespace detail {
    template<typename Terminal>
    void main() {
        if constexpr(!std::is_same<Terminal, void>::value) {
            Terminal::device_type::template init<2400>();
            std::outl<Terminal>("ConstantRate"_pgm);
        }
        
        systemTimer::template prescale<LocalConfig::tsd.prescaler>();
        systemTimer::template ocra<LocalConfig::tsd.ocr - 1>();
        systemTimer::mode(AVR::TimerMode::CTC);
        systemTimer::start();
        
        {
            Scoped<EnableInterrupt<>> ei;
            while(true)  {
                sampler::periodic();
            }
        }
    }
}

int main() {
    isrReg::init();
    MCU::Ressource::Registrar<sampler>::init();
    detail::main<terminal>();
}

// besser einen nachträglichen parser schreiben, der das unnütze push/pop wieder entfernt

//ISR(TIMER0_COMPA_vect, ISR_NAKED) { 
ISR(TIMER0_COMPA_vect) {
    isrReg::isr<AVR::ISR::Timer<0>::CompareA>();
//    Test::periodic();
//    isrReg::isrNaked<AVR::ISR::Timer<0>::CompareA>();
}
