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

#include "mcu/avr8.h"
#include "mcu/avr/util.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/ports.h"
#include "mcu/avr/isr.h"
#include "hal/event.h"
#include "units/percent.h"

template<typename InPin1, typename InPin2, typename ErrPin, typename MCUTimer>
class TLE5205 {
public:
    struct Direction {
        bool isClockWise = true;
    };
    
    typedef InPin1 pin1_pin;
    typedef InPin2 pin2_pin;
    typedef ErrPin err_pin;
    typedef MCUTimer mcu_timer_type;
    typedef typename MCUTimer::value_type value_type;
 
    static_assert(MCUTimer::hasOcrA, "need ocra");
    static_assert(MCUTimer::hasOverflow, "need overflow");
    
    using pinset = AVR::PinSet<InPin1, InPin2>;
    
    struct PwmOnHandler : public IsrBaseHandler<typename AVR::ISR::Timer<MCUTimer::number>::Overflow> {
        static void isr() {
            if (direction().isClockWise) {
                pinset::template off<InPin1, InPin2>();
            }
            else {
                pinset::template off<InPin1>();
            }
        }
    };
    struct PwmOffHandler : public IsrBaseHandler<typename AVR::ISR::Timer<MCUTimer::number>::CompareA> {
        static void isr() {
            pinset::template on<InPin1, InPin2>();
        }
    };
    
    static Direction& direction() {
        static Direction dir;
        return dir;
    }
    static void pwm(const std::percent& p) {
        using namespace std::literals::quantity;
        if (p > 0_ppc) {
            MCUTimer::mcuInterrupts()->timsk |= _BV(OCIE0A) | _BV(TOIE0);
            MCUTimer::ocra(std::expand(p, value_type{0}, std::numeric_limits<value_type>::max()));        
        }
        else {
            MCUTimer::mcuInterrupts()->timsk &= ~(_BV(OCIE0A) | _BV(TOIE0));
        }
    }
    
    template<const std::hertz& MinFrequency>
    static void init() {
        constexpr auto prescaler = AVR::Util::prescalerForAbove<MCUTimer>(MinFrequency);
        static_assert(prescaler > 0, "wrong prescaler");
        MCUTimer::template prescale<prescaler>();
        
        MCUTimer::mode(AVR::TimerMode::Normal);        
        MCUTimer::ocra(0);        
        pinset::template dir<AVR::Output>();
        pinset::allOn();
        err_pin::template dir<AVR::Input>();
        err_pin::pullup();
    }
    
    static void periodic() {
        if (!err_pin::read()) {
            // todo: Status: direction , error in ein byte
            EventManager::enqueue({EventType::TLE5205Error, 0}); 
        }
    }
private:
};

