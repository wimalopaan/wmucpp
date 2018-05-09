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

#define NDEBUG

//#define USE_INTERRUPTS
#define USE_8_BUTTONS
#define USE_DIFF_PORT

#include <stdlib.h>

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/mcutimer.h"
#include "hal/alarmtimer.h"
#include "hal/button.h"
#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using buttonPin0 = AVR::Pin<PortB, 0>;
using buttonPin1 = AVR::Pin<PortB, 1>;
#ifdef USE_DIFF_PORT
using buttonPin2 = AVR::Pin<PortC, 2>;
#else
using buttonPin2 = AVR::Pin<PortB, 2>;
#endif
using buttonPin3 = AVR::Pin<PortB, 3>;
using buttonPin4 = AVR::Pin<PortB, 4>;
using buttonPin5 = AVR::Pin<PortB, 5>;
using buttonPin6 = AVR::Pin<PortB, 6>;
using buttonPin7 = AVR::Pin<PortB, 7>;
using button0 = Button<0, buttonPin0, UseEvents<false>>;
using button1 = Button<1, buttonPin1, UseEvents<false>>;
using button2 = Button<1, buttonPin2, UseEvents<false>>;
using button3 = Button<1, buttonPin3, UseEvents<false>>;
using button4 = Button<1, buttonPin4, UseEvents<false>>;
using button5 = Button<1, buttonPin5, UseEvents<false>>;
using button6 = Button<1, buttonPin6, UseEvents<false>>;
using button7 = Button<1, buttonPin7, UseEvents<false>>;
using buttonController = ButtonController<button0, button1, button2
#ifdef USE_8_BUTTONS
, button3
, button4
, button5
, button6
, button7
#endif
>;

using ledPort = PortA;
using led0 = AVR::Pin<PortA, 0>;
using led1 = AVR::Pin<PortA, 1>;
using led2 = AVR::Pin<PortA, 2>;

// Timer0
using systemTimer = AVR::Timer8Bit<0>;

static constexpr auto f = 100_Hz;

#ifdef USE_INTERRUPTS
using bci = ButtonControllerIsr<buttonController, AVR::ISR::Timer<0>::CompareA>;
using isrRegistrar = IsrRegistrar<bci>;
#endif

int main() {
#ifdef USE_INTERRUPTS
    isrRegistrar::init();
#endif
    buttonController::init();
    led0::dir<AVR::Output>();
    led1::dir<AVR::Output>();
    led2::dir<AVR::Output>();
    
#ifdef USE_INTERRUPTS
    systemTimer::setup<f>(AVR::TimerMode::CTC);
    {
        Scoped<EnableInterrupt<>> ei;
        while(true) {
            if (bci::isPressed<button0>()) {
                led0::toggle();
            }    
            if (bci::isPressed<button1>()) {
                led1::toggle();
            }    
            if (bci::isPressed<button2>()) {
                led2::toggle();
            }    
#ifdef USE_8_BUTTONS
            if (bci::isPressed<button3>()) {
                led0::toggle();
            }    
            if (bci::isPressed<button4>()) {
                led1::toggle();
            }    
            if (bci::isPressed<button5>()) {
                led2::toggle();
            }    
            if (bci::isPressed<button6>()) {
                led0::toggle();
            }    
            if (bci::isPressed<button7>()) {
                led1::toggle();
            }    
#endif
        }
    }
#else
    systemTimer::setup<f>(AVR::TimerMode::CTCNoInt);
    while(true) {
        systemTimer::periodic<systemTimer::flags_type::ocfa>([](){
            buttonController::periodic([](uint8_t b){
                if (b == 0) {
                    led0::toggle();
                }
                if (b == 1) {
                    led1::toggle();
                }
                if (b == 2) {
                    led2::toggle();
                }
#ifdef USE_8_BUTTONS
                if (b == 3) {
                    led0::toggle();
                }
                if (b == 4) {
                    led1::toggle();
                }
                if (b == 5) {
                    led2::toggle();
                }
                if (b == 6) {
                    led0::toggle();
                }
                if (b == 7) {
                    led1::toggle();
                }
#endif
            });
        });
    }
#endif
}
#ifdef USE_INTERRUPTS
ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}
#endif
#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    while(true) {}
}
#endif
