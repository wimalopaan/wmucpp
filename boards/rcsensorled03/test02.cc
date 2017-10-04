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

#define MEM
//#define NDEBUG

#include "local.h"
#include "rcsensorled03.h"
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
    static constexpr auto title = "Test 02 Board 03"_pgm;
}

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

using isrRegistrar = IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler>;

struct Usart0Handler : public EventHandler<EventType::UsartRecv0> {
    inline static bool process(std::byte) {
        std::outl<terminal>("*** U0"_pgm);
        statusLed::blink(Blue{32}, 2);
        return true;
    }
};
struct Usart1Handler : public EventHandler<EventType::UsartRecv1> {
    static bool process(std::byte) {
        std::outl<terminal>("*** U1"_pgm);
        statusLed::blink(Blue{32}, 3);
        return true;
    }
};
struct UsartFeHandler : public EventHandler<EventType::UsartFe> {
    inline static bool process(std::byte) {
        std::outl<terminal>("*** U Fe"_pgm);
        statusLed::blink(Color{Red{32}, Green{0}, Blue{32}}, 3);
        return true;
    }
};
struct UsartUpeHandler : public EventHandler<EventType::UsartUpe> {
    inline static bool process(std::byte) {
        std::outl<terminal>("*** U UP"_pgm);
        statusLed::blink(Color{Red{32}, Green{0}, Blue{32}}, 4);
        return true;
    }
};
struct UsartDorHandler : public EventHandler<EventType::UsartDor> {
    inline static bool process(std::byte) {
        std::outl<terminal>("*** U Do"_pgm);
        statusLed::blink(Color{Red{32}, Green{0}, Blue{32}}, 5);
        return true;
    }
};

const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);

struct TimerHandler : public EventHandler<EventType::Timer> {
    inline static bool process(std::byte b) {
        auto timer = std::to_integer<uint7_t>(b);
        if (timer == *periodicTimer) {
            ++mCounter;
            statusLed::tick();

            std::outl<terminal>(Constants::title);

            const auto upm1 = rpm1::rpm();
            std::outl<terminal>(upm1.value());
            const auto upm2 = rpm2::rpm();
            std::outl<terminal>(upm2.value());
                        
            rpm1::check();
            rpm2::check();
            return true;
        }
        return false;
    }
    inline static uint8_t mCounter = 0;
};

using distributor = Distributor<isrRegistrar, statusLed, rpm1, rpm2>;

int main() {
    using namespace std::literals::quantity;
    distributor::init();
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 

    led::init();
    led::set(Constants::cGreen);
    
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    static_assert(fCr == Config::Timer::frequency);
    
    rcUsart::init<115200>();

    using allEventHandler = EventHandlerGroup<
    TimerHandler, UsartFeHandler, UsartUpeHandler, UsartDorHandler, Usart0Handler, Usart1Handler
    >;

    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>(Constants::title);

        EventManager::run2<allEventHandler>([](){
            
            adcController::periodic();
            systemClock::periodic<systemClock::flags_type::ocfa>([](){
                alarmTimer::rateProcess();
            });
            
            rpm1::periodic();
            rpm2::periodic();
            
            if (EventManager::unprocessedEvent()) {
                std::outl<terminal>("+++ Unprocessd"_pgm);
                statusLed::blink(Constants::cRed, 10);
            }
            if (EventManager::leakedEvent()) {
                std::outl<terminal>("+++ Leaked"_pgm);
                statusLed::blink(Constants::cBlue, 10);
            }
            
        });
    }    
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    while(true) {}
}
#endif

ISR(USART1_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
}
ISR(USART1_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
}
