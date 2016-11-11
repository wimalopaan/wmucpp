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

template<typename MCUTimer>
class PWM {
    typedef typename MCUTimer::mcu_type mcu_type;
    typedef MCUTimer mcu_timer_type;
    static constexpr const auto mcuTimer = MCUTimer::mcuTimer;
    static constexpr const auto mcuInterrupts = MCUTimer::mcuInterrupts;
    static constexpr auto ocMax = std::numeric_limits<typename MCUTimer::valueType>::max();
public:
    struct A {
        static void ocr(const uint16_t& v) {
            mcuTimer->ocra = v;
        }
    };
    struct B {
        static void ocr(const uint16_t& v) {
            mcuTimer->ocrb = v;
        }
    };

    static void init() {
        MCUTimer::template prescale<1024>();
        mcuTimer->tccrb |= _BV(WGM12);
        mcuInterrupts->tifr  |= _BV(OCF1A) | _BV(OCF1B);
        mcuInterrupts->timsk |= _BV(OCIE0A) | _BV(OCIE0B);
    }
    template<typename Channel>
    static void pwm(const std::percent& p) {
        typename MCUTimer::valueType v = std::expand(p, typename MCUTimer::valueType(0), std::numeric_limits<typename MCUTimer::valueType>::max());
        Channel::ocr(v);
    }
};


}
