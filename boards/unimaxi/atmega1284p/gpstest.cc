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

#define NDEBUG

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/swusart2.h"
#include "hal/alarmtimer.h"
#include "hal/softspimaster.h"
#include "hal/bufferedstream.h"
#include "external/gps/gps.h"
#include "units/duration.h"
#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using testPin1 = AVR::Pin<PortA, 3>;
using testPin2 = AVR::Pin<PortA, 2>;
using testPin3 = AVR::Pin<PortA, 1>;

// Timer0
using systemTimer = AVR::Timer8Bit<0>;
using alarmTimer = AlarmTimer<systemTimer, UseEvents<false>>;

using SoftSPIData = AVR::Pin<PortB, 1>;
using SoftSPIClock = AVR::Pin<PortB, 0>;
using SoftSPISS = AVR::Pin<PortC, 3>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS, true>;
using terminalDevice = SSpi0;
using terminal = std::basic_ostream<terminalDevice>;

using gpsPA = GPS::GpsProtocollAdapter<0, GPS::VTG, GPS::RMC>;
//using gpsUart = AVR::Usart<0, gpsPA, MCU::UseInterrupts<true>, UseEvents<false>, AVR::ReceiveQueueLength<0>>;

using gpsTx = AVR::Pin<PortA, 4>;
using gpsUartTimer = AVR::Timer8Bit<2>;
//using gpsUart = SwUsart::UsartInt<2, gpsTx, gpsPA, gpsUartTimer, 9600>; // Int2 -> PB2 -> I2CInt
using gpsUart = SwUsart::UsartInt<0, gpsTx, gpsPA, gpsUartTimer, 9600>; // Int0 -> PD2 -> RX2

using isrRegistrar = IsrRegistrar<gpsUart::RxHandler, gpsUart::TxHandler, gpsUart::StartBitHandler>;

int main() {
    testPin1::dir<AVR::Output>();
    testPin1::off();

    testPin2::dir<AVR::Output>();
    testPin2::off();
    
    testPin3::dir<AVR::Output>();
    testPin3::on();

    isrRegistrar::init();
    
    gpsUart::init();
    
    terminalDevice::init();
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 
    
    std::outl<terminal>("gpstest 01"_pgm);
    std::outl<terminal>(gpsUart::tsd.ocr);
    
    const auto periodicTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);
    
    StringBuffer<GPS::Sentence::DecimalMaxWidth> speed;
    StringBuffer<GPS::Sentence::TimeMaxWidth> time;
    StringBuffer<GPS::Sentence::DateMaxWidth> date;
    
    {
        Scoped<EnableInterrupt<>> ei;
        while(true) {
            systemTimer::periodic<systemTimer::flags_type::ocfa>([&](){
                alarmTimer::periodic([&](uint7_t timer) {
                    if (timer == *periodicTimer) {
                        GPS::VTG::speedRaw(speed);
                        std::outl<terminal>("speed: "_pgm, speed);
                        GPS::RMC::timeRaw(time);
                        std::outl<terminal>("time: "_pgm, time);
                        GPS::RMC::dateRaw(date);
                        std::outl<terminal>("date: "_pgm, date);
                    }
                });
            });
        }
    }
}

ISR(TIMER2_COMPA_vect) {
    testPin3::on();
    isrRegistrar::isr<AVR::ISR::Timer<2>::CompareA>();
    testPin3::off();
}
ISR(TIMER2_COMPB_vect) {
    testPin2::on();
    isrRegistrar::isr<AVR::ISR::Timer<2>::CompareB>();
    testPin2::off();
}
ISR(INT0_vect) {
    testPin1::on();
    isrRegistrar::isr<AVR::ISR::Int<0>>();
    testPin1::off();
}
//ISR(INT2_vect) {
//    isrRegistrar::isr<AVR::ISR::Int<2>>();
//}

//ISR(USART0_RX_vect) {
//    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
//}
//ISR(USART0_UDRE_vect){
//    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
//}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, ',', file, ',', line);
    while(true) {}
}
#endif
