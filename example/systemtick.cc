/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include "hal/softtimer.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

#include <stdlib.h>

using systemClock = AVR::Timer8Bit<0>;
using systemTimer = Timer<systemClock>;

using sampler = PeriodicGroup<systemTimer>;

using terminal = SimAVRDebugConsole;
namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

class TimerHandler : public EventHandler<EventType::Timer> {
public:
    static void process(const uint8_t&) {
        std::cout << "Hello"_pgm << std::endl;
    }
};

int main()
{
    Scoped<EnableInterrupt> interruptEnabler;
    systemTimer::init();
    auto tid1 = systemTimer::create(1000_ms, TimerFlags::Periodic);
    using handler = EventHandlerGroup<TimerHandler>;
    EventManager::run<sampler, handler>([](){});
}
ISR(TIMER0_COMPA_vect) {
    sampler::tick();
}

void assertFunction(bool b, const char* function, const char* file, unsigned int line) {
    if (!b) {
        std::cout << "Assertion failed: " << function << "," << file << "," << line << std::endl;
        abort();
    }
}
