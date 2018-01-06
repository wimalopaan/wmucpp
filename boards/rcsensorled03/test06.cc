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

struct AsciiHandler;
struct BinaryHandler;
struct BCastHandler;

using sensorUsart = AVR::Usart<0, Hott::SensorProtocollAdapter<0, UseEvents<false>, AsciiHandler, BinaryHandler, BCastHandler>, 
                               MCU::UseInterrupts<true>, UseEvents<false>> ;
using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>, MCU::UseInterrupts<true>, UseEvents<false>>;

using terminalDevice = std::conditional<useTerminal, rcUsart, void>::type;
using terminal = std::basic_ostream<terminalDevice>;

const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);

using namespace std::literals::quantity;

using isrRegistrar = IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler, 
                                  sensorUsart::RxHandler, sensorUsart::TxHandler>;

using sensorData = Hott::SensorProtocollBuffer<0>;
using crWriterSensorBinary = ConstanteRateWriter<sensorData, sensorUsart>;

struct AsciiHandler {
    static void start() {
        std::outl<terminal>("hasc start"_pgm);
    }    
    static void stop() {
        std::outl<terminal>("hasc stop"_pgm);
    }    
    static void process(std::byte) {
        
    }
};
struct BinaryHandler {
    static void start() {
        std::outl<terminal>("hbin start"_pgm);
        crWriterSensorBinary::enable<true>();
    }    
    static void stop() {
        std::outl<terminal>("hbin stop"_pgm);
        crWriterSensorBinary::enable<false>();
    }    
};
struct BCastHandler {
    static void start() {
        std::outl<terminal>("hbr start"_pgm);
    }    
    static void stop() {
        std::outl<terminal>("hbr stop"_pgm);
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

    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>("Test06"_pgm);

        while(true){
            testPin1::toggle(); // 4MHz
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
        }
    }    
}
// SumD
ISR(USART1_RX_vect) {
//    testPin2::on();
    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
//    testPin2::off(); // 1,1 us
}
ISR(USART1_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
}
// Sensor
ISR(USART0_RX_vect) {
//    testPin1::on();
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
//    testPin1::off(); // 3-4us
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

