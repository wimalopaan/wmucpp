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

#include "rcsensorled01.h"
#include "console.h"
#include "util/meta.h"
#include "appl/blink.h"

namespace {
    constexpr bool useTerminal = true;
}

namespace Constants {
    static constexpr std::hertz pwmFrequency = 100_Hz;
    static constexpr Color cOff{0};
    static constexpr Color cRed{Red{32}};
    static constexpr Color cBlue{Blue{32}};
    static constexpr Color cGreen{Green{32}};
    static constexpr auto title = "Test 13"_pgm;
}

using terminalDevice = std::conditional<useTerminal, SSpi0, void>::type;
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
            std::outl<terminal>("Test counter: "_pgm, mCounter, " Unused: "_pgm, Util::Memory::getUnusedMemory());
            statusLed::tick();
            std::outl<terminal>("channel "_pgm, (mCounter % 8), " : "_pgm, Hott::SumDProtocollAdapter<0>::value8Bit(mCounter % 8));

            std::percent pv1 = std::scale(Hott::SumDProtocollAdapter<0>::value8Bit(0));
            
            std::outl<terminal>("ch0: "_pgm, pv1);
            hbridge::pwm(pv1);
            
            std::outl<terminal>("tv: "_pgm, hbridge::timerValue());
            
            return true;
        }
        return false;
    }
    inline static uint8_t mCounter = 0;
};

using distributor = Distributor<terminalDevice, isrRegistrar, statusLed, crWriterSensorBinary, crWriterSensorText>;

int main() {
    using namespace std::literals::quantity;
    
    distributor::init();
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 
    hbridge::init<Constants::pwmFrequency>();
    hbridge::pwm(0_ppc);
    hbridge::direction() = hbridge::Direction{false};
    
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    constexpr auto tsd = AVR::Util::calculate<sensorRateTimer>(fCr);
    static_assert(tsd, "wrong parameter");
    sensorRateTimer::prescale<tsd.prescaler>();
    sensorRateTimer::ocra<tsd.ocr>();
    sensorRateTimer::mode(AVR::TimerMode::CTCNoInt);
    
    sensorUsart::init<19200>();
    rcUsart::init<115200>();

    using allEventHandler = EventHandlerGroup<
    TimerHandler, UsartFeHandler, UsartUpeHandler, UsartDorHandler, Usart0Handler, Usart1Handler, HottBinaryHandler, HottBroadcastHandler, HottTextHandler, HottKeyHandler>;

    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>(Constants::title);
        
        EventManager::run2<allEventHandler>([](){
            systemClock::periodic<systemClock::flags_type::ocfa>([](){
                alarmTimer::periodic();
            });
            sensorRateTimer::periodic<sensorRateTimer::flags_type::ocfa>([]() {
                crWriterSensorBinary::rateProcess();
                crWriterSensorText::rateProcess();
            });
            if (EventManager::unprocessedEvent()) {
                statusLed::blink(Constants::cRed, 10);
            }
            if (EventManager::leakedEvent()) {
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
