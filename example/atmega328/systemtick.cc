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

#include "mcu/ports.h"
#include "mcu/avr/isr.h"
#include "hal/alarmtimer.h"
#include "simavr/simavrdebugconsole.h"
#include "util/dassert.h"
#include "console.h"

using systemClock= AVR::Timer8Bit<0>;
using systemTimer = AlarmTimer<systemClock>;

using sampler = PeriodicGroup<AVR::ISR::Timer<0>::CompareA, systemTimer>;

using isrReg = IsrRegistrar<sampler>;

using terminal = SimAVRDebugConsole;
namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

class TimerHandler : public EventHandler<EventType::Timer> {
public:
    static void process(uint8_t) {
        std::cout << "Hello"_pgm << std::endl;
    }
};

int main()
{
    Scoped<EnableInterrupt> interruptEnabler;
    systemTimer::init();
    auto tid1 = systemTimer::create(1000_ms, AlarmFlags::Periodic);
    using handler = EventHandlerGroup<TimerHandler>;
    EventManager::run<sampler, handler>([](){});
}

ISR(TIMER0_COMPA_vect) {
    isrReg::isr<AVR::ISR::Timer<0>::CompareA>();
//    sampler::tick();
}

#ifndef NDEBUG
void assertFunction(const char* e, const char* function, const char* file, unsigned int line) {
    std::cout << "Assertion failed: "_pgm << e << function << ","_pgm << file << ","_pgm << line << std::endl;
    while(true) {}
}
#endif
