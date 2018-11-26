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

// Clock = 8MHz / 8 = 1MHz, EEProm 
// sudo avrdude -p attiny85 -P usb -c avrisp2 -U lfuse:w:0x62:m -U hfuse:w:0xd7:m -U efuse:w:0xff:m

// sudo avrdude -p attiny85 -P usb -c avrisp2 -U flash:w:main.elf

#define NDEBUG

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/sleep.h"
#include "mcu/avr/groups.h"
#include "hal/alarmtimer.h"
#include "util/disable.h"
#include "util/types.h"

template<auto T>
struct static_print {
    std::integral_constant<uint16_t, T> v;
    using type = typename decltype(v)::_;
};

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

using systemTimer = AVR::Timer8Bit<0>;
using alarmTimer  = AlarmTimer<systemTimer, UseEvents<false>>;

using toneTimer = AVR::Timer8Bit<1>;

using mosfetPin = AVR::ActiveHigh<AVR::Pin<PortB, 4>, AVR::Output>;
using buttonPin = AVR::ActiveLow<AVR::Pin<PortB, 2>, AVR::Input>;
using buzzerPin1 = AVR::Pin<PortB, 1>;
using buzzerPin0 = AVR::Pin<PortB, 0>;
using testPin   = AVR::Pin<PortB, 3>;

struct Parameter {
    inline static constexpr auto intervall = 100_ms;
    inline static constexpr uint8_t ticksPerSecond = 1000_ms / intervall;
};

using sleep = AVR::Sleep<>;

inline static void softStart() {
//    for(uint8_t counter = 0; counter < 100; ++counter) {
    for(;;) {
        mosfetPin::activate();        
        Util::delay(5_us); // 20us on
        mosfetPin::inactivate();        
        Util::delay(200_us); // 200us off -> 20/220 = 1/11 PWM
    }
    mosfetPin::activate();        
}

int main() {
    mosfetPin::init();
    buttonPin::init();
    
    testPin::dir<AVR::Output>();
    testPin::off();
    
    systemTimer::setup<Config::Timer::frequency>(AVR::TimerMode::CTCNoInt);
    auto secondsTimer = alarmTimer::create(Parameter::intervall, AlarmFlags::Periodic);
    
    {
        Scoped<EnableInterrupt<>> ei;

        softStart();
        
        while(true) {
            systemTimer::periodic<systemTimer::flags_type::ocf0a>([&](){
                alarmTimer::periodic([&](uint7_t timer){
                    if (timer == *secondsTimer) {
                        testPin::toggle();
                    }                
                });
            });
        }
    }
}


#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
    while(true) {
        led::toggle();
    }
}
#endif
