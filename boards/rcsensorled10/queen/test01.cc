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


#define MEM
#define NDEBUG
#define OUTPUT

#include "local.h"
#include "rcsensorled10.h"
#include "console.h"

using testPin0 = AVR::Pin<PortA, 5>; // ACS1
using testPin1 = AVR::Pin<PortA, 6>; // ACS2
using testPin2 = AVR::Pin<PortA, 7>; // ACS3

namespace Constants {
    static constexpr std::hertz pwmFrequency = 100_Hz * 256; 
    static constexpr const std::hertz fSCL = 100000_Hz;
}

struct AsciiHandler;
struct BinaryHandler;
struct BCastHandler;

using sensorUsart = AVR::Usart<0, Hott::SensorProtocollAdapter<0, UseEvents<false>, AsciiHandler, BinaryHandler, BCastHandler>, 
MCU::UseInterrupts<true>, UseEvents<false>> ;
using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>, MCU::UseInterrupts<true>, UseEvents<false>>;

using terminalDevice = rcUsart;
using terminal = std::basic_ostream<terminalDevice>;

using namespace std::literals::quantity;

using isrRegistrar = IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler
, gpsUsart::RxHandler, gpsUsart::TxHandler
, sensorUsart::RxHandler, sensorUsart::TxHandler
>;

using sensorData = Hott::SensorProtocollBuffer<0>;
using crWriterSensorBinary = ConstanteRateWriter<sensorData, sensorUsart>;
using menuData = Hott::SensorTextProtocollBuffer<0>;
using crWriterSensorText = ConstanteRateWriter<menuData, sensorUsart>;


struct AsciiHandler {
    static void start() {
        crWriterSensorText::enable<true>();
    }    
    static void stop() {
        // not: das disable sollte automatisch laufen
        //        crWriterSensorText::enable<false>();
    }    
    static void process(std::byte ) {
    }
};
struct BinaryHandler {
    static void start() {
        crWriterSensorBinary::enable<true>();
    }    
    static void stop() {
        // not: das disable sollte automatisch laufen
        //        crWriterSensorBinary::enable<false>();
    }    
};
struct BCastHandler {
    static void start() {
#ifdef OUTPUT
        std::outl<terminal>("hbr start"_pgm);
#endif
        crWriterSensorBinary::enable<true>();
    }    
    static void stop() {
    }    
};

struct Storage {
    inline static StringBuffer<GPS::Sentence::TimeMaxWidth> time;
    inline static StringBuffer<GPS::Sentence::DecimalMaxWidth> decimalBuffer;
};

int main() {
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    static_assert(fCr == Config::Timer::frequency);
    
    testPin0::dir<AVR::Output>();
    testPin0::off();
    testPin1::dir<AVR::Output>();
    testPin1::off();
    testPin2::dir<AVR::Output>();
    testPin2::off();
    
    isrRegistrar::init();

    alarmTimer::init(AVR::TimerMode::CTCNoInt); 
    
    hardPwm::init<Constants::pwmFrequency>();
    hbridge1::init();
    hbridge2::init();
    
    crWriterSensorBinary::init();
    crWriterSensorText::init();
    
    gpsUsart::init<9600>();
    sensorUsart::init<19200>();
    rcUsart::init<115200>();
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>("Queen01"_pgm);
        
        const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);

        while(true){
            auto v = Hott::SumDProtocollAdapter<0>::value(0);
            hbridge1::pwm(v);
            hbridge2::pwm(v);
            systemClock::periodic<systemClock::flags_type::ocfa>([&](){
                testPin0::on();
                crWriterSensorBinary::rateProcess();
                crWriterSensorText::rateProcess();
                testPin0::off();
                alarmTimer::periodic([&](uint7_t timer){
                    if (timer == *periodicTimer) {
                        GPS::RMC::timeRaw(Storage::time);
                        GPS::VTG::speedRaw(Storage::decimalBuffer);
//                        auto s = Util::StringConverter<FixedPoint<int16_t, 4>>::parse(Storage::decimalBuffer);
//                        std::outl<terminal>("mem: "_pgm, Util::Memory::getUnusedMemory());
                        std::outl<terminal>("time: "_pgm, Storage::time);
//                        std::outl<terminal>("speed: "_pgm, s);
                        std::outl<terminal>("v: "_pgm, v.toInt());
//                        auto valid = Hott::SumDProtocollAdapter<0>::valid();
//                        std::outl<terminal>("valid: "_pgm, valid);
                        
                    }
                });
           });
        }
    }
}
// GPS
ISR(USART2_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<2>::RX>();
}
ISR(USART2_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<2>::UDREmpty>();
}
// SumD
ISR(USART1_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
}
ISR(USART1_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
}
// Sensor
ISR(USART0_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART0_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
}
#endif
