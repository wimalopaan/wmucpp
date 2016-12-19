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
#include "mcu/ports.h"
#include "mcu/avr/isr.h"

template<typename InPin1, typename InPin2, typename ErrPin, typename MCUTimer>
class TLE5205 {
public:
    class PwmOnHandler : public IsrBaseHandler<typename AVR::ISR::Timer<MCUTimer::N>::CompareA> {
        
    };
    class PwmOffHandler : public IsrBaseHandler<typename AVR::ISR::Timer<MCUTimer::N>::Overflow> {
        
    };
    
    typedef InPin1 pin1_pin;
    typedef InPin2 pin2_pin;
    typedef ErrPin err_pin;
    typedef MCUTimer mcu_timer_type;
 
    using pinset = AVR::PinSet<InPin1, InPin2>;
    
    static constexpr uint8_t states[] = {InPin1::mask | InPin2::mask, InPin1::mask, InPin2::mask};
    
    
    template<const std::hertz& MinFrequency>
    static void init() {
        constexpr auto prescaler = AVR::Util::prescalerForAbove<MCUTimer>(MinFrequency);
        static_assert(prescaler > 0, "wrong prescaler");
        MCUTimer::template prescale<prescaler>();
        
        pinset::allOff();
        pinset::template on<pin1_pin>();
    }
private:
    static bool directionCW;
};

template<typename InPin1, typename InPin2, typename ErrPin, typename MCUTimer>
bool TLE5205<InPin1, InPin2, ErrPin, MCUTimer>::directionCW = true;