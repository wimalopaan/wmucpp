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

#include <stdlib.h>

#include "simavr.h"
#include "mcu/avr8.h"
#include "util/dassert.h"
#include "mcu/ports.h"
#include "hal/alarmtimer.h"
#include "util/disable.h"
#include "hal/event.h"
#include "mcu/avr/ppm.h"
#include "mcu/avr//pinchange.h"
#include "hal/ppmswitch.h"
#include "container/pgmstring.h"
#include "container/stringbuffer.h"
#include "hal/softppm.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

using systemClock = AVR::Timer8Bit<0>;
using systemTimer = AlarmTimer<systemClock>;

using terminal = SimAVRDebugConsole;

using sampler = PeriodicGroup<AVR::ISR::Timer<0>::CompareA, systemTimer>;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}
#include "test/simpletest.h"

SIMPLETEST("a") {
    return true;
};

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;


using ppmInputPin = AVR::Pin<PortC, 0>;
using ppmInputset = AVR::PinSet<ppmInputPin>;
using pinChangeHandlerPpm = AVR::PinChange<ppmInputset>;
using ppmTimer = AVR::Timer8Bit<2>;
using ppm1 = PpmDecoder<pinChangeHandlerPpm, ppmTimer>;
using ppmSwitch = PpmSwitch<0, ppm1>;

//using pwmTimer = AVR::Timer16Bit<1>;
//using pwmPin = AVR::Pin<PortD, 6>;
//using softPwm = SoftPPM<pwmTimer, pwmPin>;

using constantRateTimer = AVR::Timer16Bit<1>;

using isrReg = IsrRegistrar<sampler>;


struct EventHandlerParameter {
    std::optional<uint7_t> timerId1;
    std::optional<uint7_t> timerId2;
};

EventHandlerParameter evp;

class TimerHandler : public EventHandler<EventType::Timer> {
public:
    static void process(const uint8_t& tid) {
        if (tid == *evp.timerId1) {
            std::cout << "timer1"_pgm << std::endl;
        }
        if (tid == *evp.timerId2) {
            std::cout << "timer2"_pgm << std::endl;
        }
    }
};

void assertFunction(bool b, const char* function, const char* file, unsigned int line) {
    if (!b) {
        std::cout << "Assertion failed: " << function << "," << file << "," << line << std::endl;
        abort();
    }
}

int main(void) {
    std::cout << "simavr"_pgm << std::endl;

    Scoped<EnableInterrupt> interruptEnabler;

    constexpr auto t = AVR::Util::calculate<systemClock>(Config::Timer::frequency);
    static_assert(t, "falscher wert für p");
    std::cout << "pre: "_pgm << t.prescaler << std::endl;
    std::cout << "ocr: "_pgm << t.ocr << std::endl;

    constexpr const std::hertz f = 1 / 1600_us;
    constexpr auto crt = AVR::Util::calculate<constantRateTimer>(f);
    static_assert(crt, "falscher wert für p");
    std::cout << "crt f: "_pgm << f << std::endl;
    std::cout << "crt pre: "_pgm << crt.prescaler << std::endl;
    std::cout << "crt ocr: "_pgm << crt.ocr << std::endl;

    uint32_t a = 1000;
    uint32_t b = 1000;
    uint32_t x = a * b;

    std::cout << "x::: " << x << std::endl;

    std::cout << Config() << std::endl;

    systemTimer::init();

    pinChangeHandlerPpm::init();
    PpmDecoder<pinChangeHandlerPpm, ppmTimer>::init();

    std::cout << "ppmtimer: "_pgm << ppm1::timerFrequency << std::endl;
    std::cout << "prescaler: "_pgm << ppm1::prescaler << std::endl;
    std::cout << "ppmMin: "_pgm << ppm1::ppmMin << std::endl;
    std::cout << "ppmMax: "_pgm << ppm1::ppmMax << std::endl;
    std::cout << "ppmMid: "_pgm << ppm1::ppmMid << std::endl;
    std::cout << "ppmDelta: "_pgm << ppm1::ppmDelta << std::endl;
    std::cout << "ppmMidLow: "_pgm << ppm1::ppmMidLow << std::endl;
    std::cout << "ppmMidHigh: "_pgm << ppm1::ppmMidHigh << std::endl;
    std::cout << "ppmMaxLow: "_pgm << ppm1::ppmMaxLow << std::endl;
    std::cout << "ppmMinHigh: "_pgm << ppm1::ppmMinHigh << std::endl;

//    std::cout << "softPwm p: "_pgm << softPwm::prescaler << std::endl;
//    std::cout << "softPwm ocmin: "_pgm << softPwm::ocMin << std::endl;
//    std::cout << "softPwm ocmax: "_pgm << softPwm::ocMax << std::endl;


    evp.timerId1 = systemTimer::create(1000_ms, AlarmFlags::Periodic);
    evp.timerId2 = systemTimer::create(10_s, AlarmFlags::Periodic);

    using handler = EventHandlerGroup<TimerHandler>;

    EventManager::run<sampler, handler>();
}

ISR(TIMER0_COMPA_vect) {
    isrReg::isr<AVR::ISR::Timer<0>::CompareA>();
//    sampler::tick();
}

// todo: isrReg

ISR(TIMER1_COMPA_vect) {
//    softPwm::isrA();
}

ISR(TIMER1_COMPB_vect) {
//    softPwm::isrB();
}
