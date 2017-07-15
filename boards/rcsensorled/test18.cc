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
#define NDEBUG

#include "local02.h"
#include "rcsensorled01.h"
#include "console.h"
#include "util/meta.h"
#include "appl/blink.h"

namespace {
    constexpr bool useTerminal = true;
}

namespace Constants {
    static constexpr Color cOff{0};
    static constexpr Color cRed{Red{32}};
    static constexpr Color cBlue{Blue{32}};
    static constexpr Color cGreen{Green{32}};
    static constexpr auto title = "Test 12"_pgm;
}

//using terminalDevice = std::conditional<useTerminal, SSpi0, void>::type;
using terminalDevice = rcUsart;
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

using sensorData = Hott::SensorProtocollBuffer<0>;
using menuData = Hott::SensorTextProtocollBuffer<0>;
using crWriterSensorBinary = ConstanteRateWriter<sensorData, sensorUsart>;
using crWriterSensorText = ConstanteRateWriter<menuData, sensorUsart>;

using isrRegistrar = IsrRegistrar<sensorUsart::RxHandler, sensorUsart::TxHandler, rcUsart::RxHandler, rcUsart::TxHandler>;

struct HottBinaryHandler : public EventHandler<EventType::HottBinaryRequest> {
    inline static bool process(std::byte) {
        crWriterSensorBinary::enable<true>();
        crWriterSensorText::enable<false>();
        return true;
    }
};

struct HottKeyHandler : public EventHandler<EventType::HottAsciiKey> {
    inline static bool process(std::byte v) {
        std::outl<terminal>("key: "_pgm, v);
        return true;
    }
};

struct HottBroadcastHandler : public EventHandler<EventType::HottSensorBroadcast> {
    inline static bool process(std::byte) {
        std::outl<terminal>("hbr"_pgm);
        crWriterSensorBinary::enable<true>();
        crWriterSensorText::enable<false>();
        return true;
    }
};

struct HottTextHandler : public EventHandler<EventType::HottAsciiRequest> {
    inline static bool process(std::byte) {
        std::outl<terminal>("hba"_pgm);
        crWriterSensorBinary::enable<false>();
        crWriterSensorText::enable<true>();
        return true;
    }
};
struct Usart0Handler : public EventHandler<EventType::UsartRecv0> {
    inline static bool process(std::byte) {
        statusLed::blink(Blue{32}, 2);
        return true;
    }
};
struct Usart1Handler : public EventHandler<EventType::UsartRecv1> {
    static bool process(std::byte) {
        statusLed::blink(Blue{32}, 3);
        return true;
    }
};
struct UsartFeHandler : public EventHandler<EventType::UsartFe> {
    inline static bool process(std::byte) {
        statusLed::blink(Color{Red{32}, Green{0}, Blue{32}}, 3);
        return true;
    }
};
struct UsartUpeHandler : public EventHandler<EventType::UsartUpe> {
    inline static bool process(std::byte) {
        statusLed::blink(Color{Red{32}, Green{0}, Blue{32}}, 4);
        return true;
    }
};
struct UsartDorHandler : public EventHandler<EventType::UsartDor> {
    inline static bool process(std::byte) {
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
            std::outl<terminal>("channel "_pgm, (mCounter % 8), " : "_pgm, Hott::SumDProtocollAdapter<0>::value8Bit(mCounter % 8));
            
            auto v1 = Hott::SumDProtocollAdapter<0>::value(2);
            hardPpm::ppm<hardPpm::A>(v1);
            auto v2 = Hott::SumDProtocollAdapter<0>::value(3);
            hardPpm::ppm<hardPpm::B>(v2);
            
            return true;
        }
        return false;
    }
    inline static uint8_t mCounter = 0;
};

using distributor = Distributor<isrRegistrar, statusLed, crWriterSensorBinary, crWriterSensorText, leds1, leds2, hardPpm>;

int main() {
    distributor::init();
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 

    leds1::set(Constants::cGreen);
    leds2::set(Constants::cRed);
    
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    static_assert(fCr == Config::Timer::frequency);
    
    sensorUsart::init<19200>();
    rcUsart::init<115200>();

    using allEventHandler = EventHandlerGroup<
    TimerHandler, UsartFeHandler, UsartUpeHandler, UsartDorHandler, Usart0Handler, HottBinaryHandler, HottBroadcastHandler, HottTextHandler, HottKeyHandler>;

    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>(Constants::title);
        
        EventManager::run2<allEventHandler>([](){
            systemClock::periodic<systemClock::flags_type::ocfa>([](){
                alarmTimer::rateProcess();
                crWriterSensorBinary::rateProcess();
                crWriterSensorText::rateProcess();
            });

            if (EventManager::unprocessedEvent()) {
                statusLed::blink(Constants::cRed, 3);
            }
            if (EventManager::leakedEvent()) {
                statusLed::blink(Constants::cBlue, 3);
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

// Timer 2
// SystemClock
//ISR(TIMER2_COMPA_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<2>::CompareA>();
//}
// Timer 4
// Sensor Constant Rate
//ISR(TIMER4_COMPA_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<4>::CompareA>();
//}

// Sensor
ISR(USART0_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART0_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}
// SumD
ISR(USART1_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
}
ISR(USART1_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
}
