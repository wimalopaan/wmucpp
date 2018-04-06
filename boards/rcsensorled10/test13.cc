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
#define OUTPUT

#include "local.h"
#include "rcsensorled10.h"
#include "console.h"

//using testPin0 = AVR::Pin<PortA, 0>; // LipoA1 = Chan0
//using testPin1 = AVR::Pin<PortA, 1>; // LipoA2 = Chan1
//using testPin2 = AVR::Pin<PortA, 2>; // LipoA3 = Chan1
using testPin0 = AVR::Pin<PortA, 5>; // ACS1
using testPin1 = AVR::Pin<PortA, 6>; // ACS2
using testPin2 = AVR::Pin<PortA, 7>; // ACS3

namespace Constants {
    static constexpr std::hertz pwmFrequency = 100_Hz * 256; 
    static constexpr const std::hertz fSCL = 100000_Hz;
}

using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>, MCU::UseInterrupts<true>, UseEvents<false>>;

using terminalDevice = rcUsart;
using terminal = std::basic_ostream<terminalDevice>;

using namespace std::literals::quantity;

using isrRegistrar = IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler
, gpsUsart::RxHandler, gpsUsart::TxHandler
>;

int main() {
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    static_assert(fCr == Config::Timer::frequency);
    
    testPin0::dir<AVR::Output>();
    testPin0::on();
    testPin1::dir<AVR::Output>();
    testPin1::on();
    testPin2::dir<AVR::Output>();
    testPin2::on();
    
    isrRegistrar::init();

    alarmTimer::init(AVR::TimerMode::CTCNoInt); 
    
    hardPwm::init<Constants::pwmFrequency>();
    hbridge1::init();
    hbridge2::init();
    
    gpsUsart::init<9600>();
    rcUsart::init<115200>();
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>("Test13"_pgm);
        
        const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);

        while(true){
            testPin0::toggle();
            auto v = Hott::SumDProtocollAdapter<0>::value(0);
            hbridge1::pwm(v);
            hbridge2::pwm(v);
            systemClock::periodic<systemClock::flags_type::ocfa>([&](){
                alarmTimer::periodic([&](uint7_t timer){
                    if (timer == *periodicTimer) {
                        StringBuffer<GPS::Sentence::TimeMaxWidth> time;
                        GPS::RMC::timeRaw(time);
                        std::outl<terminal>("time: "_pgm, time);
                        std::outl<terminal>("mem: "_pgm, Util::Memory::getUnusedMemory());
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

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    while(true) {}
}
#endif
