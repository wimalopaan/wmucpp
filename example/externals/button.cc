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

#include <stdlib.h>

//#include "main.h"

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/mcutimer.h"
#include "hal/alarmtimer.h"
#include "hal/button.h"
#include "console.h"

// 16 MHz full swing
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xf7:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

// 8Mhz int
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using buttonPin0 = AVR::Pin<PortB, 0>;
using buttonPin1 = AVR::Pin<PortB, 1>;
using buttonPin2 = AVR::Pin<PortB, 2>;
using button0 = Button<0, buttonPin0, UseEvents<false>>;
using button1 = Button<1, buttonPin1, UseEvents<false>>;
using button2 = Button<1, buttonPin2, UseEvents<false>>;
using buttonController = ButtonController<button0, button1, button2>;

using testPin = AVR::Pin<PortA, 0>;

// Timer0
using systemTimer = AVR::Timer8Bit<0>;
using alarmTimer = AlarmTimer<systemTimer, UseEvents<false>>;

//using flagRegister = AVR::RegisterFlags<typename DefaultMcuType::GPIOR, 0, std::byte>;

int main() {
    buttonController::init();
    testPin::dir<AVR::Output>();
    
    alarmTimer::init(AVR::TimerMode::CTCNoInt);

    while(true) {
        systemTimer::periodic<systemTimer::flags_type::ocfa>([](){
            buttonController::periodic([](uint8_t b){
                if (b == 0) {
                    testPin::toggle();
                }
            });
//            alarmTimer::periodic([](uint7_t timer) {
//            });
        });
    }
}
#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    while(true) {}
}
#endif
