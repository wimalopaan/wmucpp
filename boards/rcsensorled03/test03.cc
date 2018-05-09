/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

//#define MEM
#define NDEBUG

#include "local.h"
#include "rcsensorled03b.h"
#include "console.h"
#include "util/meta.h"
#include "appl/blink.h"

namespace {
    constexpr bool useTerminal = true;
}

namespace Constants {
    static constexpr std::hertz pwmFrequency = 100_Hz * 256;
    static constexpr Color cOff{0};
    static constexpr Color cRed{Red{32}};
    static constexpr Color cBlue{Blue{32}};
    static constexpr Color cGreen{Green{32}};
    static constexpr auto title = "Test 03 Board 03"_pgm;
}

using sensorUsart = AVR::Usart<0, void, MCU::UseInterrupts<false>, UseEvents<false>, AVR::ReceiveQueueLength<32>> ;
using rcUsart = AVR::Usart<1, void, MCU::UseInterrupts<false>, UseEvents<false>, AVR::ReceiveQueueLength<32>>;

using terminalDevice = std::conditional<useTerminal, rcUsart, void>::type;
using terminal = std::basic_ostream<terminalDevice>;

using statusLed = Blinker<led, Constants::cGreen>;

template<typename... T>
struct Distributor {
    using Items = Meta::filter<Meta::nonVoid, Meta::List<T...>>;
    template<typename U> struct NonVoidDistributor;
    template<template<typename...> typename L, typename... U>
    struct NonVoidDistributor<L<U...>> {
        inline static void init() {
            (U::init(), ...);
        }
    };
    inline static void init() {
        NonVoidDistributor<Items>::init();
    }
};

const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);

using distributor = Distributor<statusLed, rpm1, rpm2>;

int main() {
    using namespace std::literals::quantity;
    distributor::init();
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 

    Util::delay(10_ms);
    
    led::init();
    led::set(Constants::cGreen);
    
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    static_assert(fCr == Config::Timer::frequency);
    
    sensorUsart::init<19200>();
    rcUsart::init<115200>();
    

    {
//        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>(Constants::title);
        std::outl<terminal>("---"_pgm);

        std::outl<terminal>(rpm1::fTimer);
        std::outl<terminal>(rpm1::prescaler);

        std::outl<terminal>("---"_pgm);

        std::outl<terminal>(softPpm::parameter::prescaler);
        std::outl<terminal>(softPpm::parameter::timerFrequency);

        while(true){
//            adcController::periodic();
//            rpm1::periodic();
//            rpm2::periodic();
            rcUsart::periodic();
            sensorUsart::periodic();
            systemClock::periodic<systemClock::flags_type::ocfa>([](){
                alarmTimer::periodic([](uint7_t timer){
                    if (timer == *periodicTimer) {
                        statusLed::tick();   
                    }
                });
            });
        }
    }    
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    while(true) {}
}
#endif

