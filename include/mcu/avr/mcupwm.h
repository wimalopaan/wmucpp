/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#pragma once

#include "mcu/avr/mcutimer.h"
#include "units/percent.h"
#include "std/limits.h"

namespace AVR {

// todo: fertig machen

template<uint8_t TimerN, typename MCU = DefaultMcuType>
struct PwmParamter;

template<>
struct PwmParamter<0, ATMega1284P> {
};
template<>
struct PwmParamter<1, ATMega1284P> {
    using PortD = AVR::Port<ATMega1284P::PortRegister, AVR::D>;
    typedef AVR::Pin<PortD, 5> pwmA;
    typedef AVR::Pin<PortD, 4> pwmB;

    typedef AVR::Timer16Bit<1, ATMega1284P> timer_type;
};
template<>
struct PwmParamter<2, ATMega1284P> {
};
template<>
struct PwmParamter<3, ATMega1284P> {
    using PortB = AVR::Port<ATMega1284P::PortRegister, AVR::B>;
    typedef AVR::Pin<PortB, 6> pwmA;
    typedef AVR::Pin<PortB, 7> pwmB;

    typedef AVR::Timer16Bit<1, ATMega1284P> timer_type;

};

template<uint8_t TimerN, typename MCU = DefaultMcuType>
class PWM {
    typedef MCU mcu_type;
    typedef typename PwmParamter<TimerN, MCU>::timer_type timer_type;
    using MCUTimer = typename timer_type::mcu_timer_type;
    static constexpr const auto mcuTimer = getBaseAddr<MCUTimer, TimerN>();
    using pwmA = typename PwmParamter<TimerN, MCU>::pwmA;
    using pwmB = typename PwmParamter<TimerN, MCU>::pwmB;
public:
    struct A {
        static void ocr(const typename timer_type::value_type& v) {
            mcuTimer->ocra = v;
        }
    };
    struct B {
        static void ocr(const typename timer_type::value_type& v) {
            mcuTimer->ocrb = v;
        }
    };

    static void init() {
        pwmA::template dir<AVR::Output>();
        pwmB::template dir<AVR::Output>();
        timer_type::template prescale<256>();
        mcuTimer->tccra |= _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0) // non-inverting mode
                                       | _BV(WGM10) | _BV(WGM11) ; // 10-bit fast PWM
        mcuTimer->tccrb |= _BV(WGM12); // 10-bit fast PWM
    }
    template<typename Channel>
    static void pwm(const std::percent& p) {
        typename timer_type::value_type v = std::expand(p, typename timer_type::value_type(0), (uint16_t)0x03ff);
        Channel::ocr(v);
    }
};


}
