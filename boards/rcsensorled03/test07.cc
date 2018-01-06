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

//#define MEM
#define NDEBUG

#include "local.h"
#include "rcsensorled03b.h"
#include "console.h"
#include "util/meta.h"

using testPin1 = AVR::Pin<PortB, 1>; // ppm1
using testPin2 = AVR::Pin<PortB, 2>; // ppm2

namespace {
    constexpr bool useTerminal = true;
}

namespace Constants {
    static constexpr Color cRed{Red{32}};
    static constexpr Color cBlue{Blue{32}};
    static constexpr Color cGreen{Green{32}};
}

using sensorUsart = AVR::Usart<0, Hott::SensorProtocollAdapter<0, UseEvents<true>>, 
MCU::UseInterrupts<true>, UseEvents<true>> ;
using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>, MCU::UseInterrupts<true>, UseEvents<true>>;

using terminalDevice = std::conditional<useTerminal, rcUsart, void>::type;
using terminal = std::basic_ostream<terminalDevice>;

const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);

using namespace std::literals::quantity;

using isrRegistrar = IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler, 
                                  sensorUsart::RxHandler, sensorUsart::TxHandler>;

using sensorData = Hott::SensorProtocollBuffer<0>;
using crWriterSensorBinary = ConstanteRateWriter<sensorData, sensorUsart>;

struct HottBinaryHandler : public EventHandler<EventType::HottBinaryRequest> {
    inline static bool process(std::byte) {
        testPin1::on();
        crWriterSensorBinary::enable<true>();
        //        crWriterSensorText::enable<false>();
        return true;
    }
};

struct HottKeyHandler : public EventHandler<EventType::HottAsciiKey> {
    inline static bool process(std::byte v) {
        std::outl<terminal>("key: "_pgm, v);
        //        menu::processKey(Hott::key_t(v));
        return true;
    }
};

struct HottBroadcastHandler : public EventHandler<EventType::HottSensorBroadcast> {
    inline static bool process(std::byte) {
        std::outl<terminal>("hbr"_pgm);
        crWriterSensorBinary::enable<true>();
        //        crWriterSensorText::enable<false>();
        return true;
    }
};

struct HottTextHandler : public EventHandler<EventType::HottAsciiRequest> {
    inline static bool process(std::byte) {
        crWriterSensorBinary::enable<false>();
        //        crWriterSensorText::enable<true>();
        return true;
    }
};
struct Usart0Handler : public EventHandler<EventType::UsartRecv0> {
    inline static bool process(std::byte) {
        return true;
    }
};
struct Usart1Handler : public EventHandler<EventType::UsartRecv1> {
    static bool process(std::byte) {
        return true;
    }
};
struct UsartFeHandler : public EventHandler<EventType::UsartFe> {
    inline static bool process(std::byte) {
        return true;
    }
};
struct UsartUpeHandler : public EventHandler<EventType::UsartUpe> {
    inline static bool process(std::byte) {
        return true;
    }
};
struct UsartDorHandler : public EventHandler<EventType::UsartDor> {
    inline static bool process(std::byte) {
        return true;
    }
};


int main() {
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    static_assert(fCr == Config::Timer::frequency);
    
    isrRegistrar::init();
    
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 
    
    led::init();
    led::set(Constants::cGreen);
    
    testPin1::dir<AVR::Output>();
    testPin2::dir<AVR::Output>();
    
    crWriterSensorBinary::init();
    
    sensorUsart::init<19200>();
    rcUsart::init<115200>();
    
    using allEventHandler = EventHandlerGroup<UsartFeHandler, UsartUpeHandler, UsartDorHandler, Usart0Handler, Usart1Handler,
                                             HottBinaryHandler, HottBroadcastHandler, HottTextHandler, HottKeyHandler>;
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>("Test07"_pgm);
        
        EventManager::run2<allEventHandler>([](){
//            testPin1::toggle(); // 500KHz
            systemClock::periodic<systemClock::flags_type::ocfa>([](){
                crWriterSensorBinary::rateProcess();
                alarmTimer::periodic([](uint7_t timer){
                    if (timer == *periodicTimer) {
                        auto v = Hott::SumDProtocollAdapter<0>::value8Bit(0);
                        std::percent pv = std::scale(v);
                        std::outl<terminal>("v: "_pgm, uint8_t(v), " : "_pgm, pv);
                    }
                });
            });
            if (EventManager::unprocessedEvent()) {
                std::outl<terminal>("+++ Unprocessd"_pgm);
            }
            if (EventManager::leakedEvent()) {
                std::outl<terminal>("+++ Leaked"_pgm);
            }
        });
    }
}
// SumD
ISR(USART1_RX_vect) {
//    testPin2::on();
    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
//    testPin2::off(); // -> 3-4 us
}
ISR(USART1_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
}
// Sensor
ISR(USART0_RX_vect) {
//    testPin1::on();
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
//    testPin1::off(); // -> 3us
}
ISR(USART0_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    while(true) {}
}
#endif

