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
#include "mcu/avr/isr.h"
#include "util/disable.h"

template<typename MCUTimer, typename ValueType>
class SoftTimer : public IsrBaseHandler<typename AVR::ISR::Timer<MCUTimer::number>::Overflow> {
    static_assert(sizeof(ValueType) > sizeof(typename MCUTimer::value_type), "ValueType must be larger than mcutimer::value_type");
    SoftTimer() = delete;
public:
    typedef typename MCUTimer::mcu_timer_type mcu_timer_type;
    typedef typename MCUTimer::mcu_type mcu_type;
    
    static constexpr uint8_t number = MCUTimer::number;
    
    typedef ValueType value_type;
    
    template<int PreScale>
    static inline void prescale() {
        MCUTimer::template prescale<PreScale>();
    }   
    static std::hertz frequency() {
        return MCUTimer::frequency();
    }

    static AVR::PrescalerPair::scale_type prescaler() {
        return MCUTimer::prescaler();
    }
    
    static inline void mode(const AVR::TimerMode& mode) {
        // fixme: flag handling
        MCUTimer::mode(mode);
        MCUTimer::mode(AVR::TimerMode::OverflowInterrupt);
    }
    
    static inline volatile ValueType counter() {
        Scoped<DisbaleInterrupt> di;
        return (mCounter << Util::numberOfBits<typename MCUTimer::value_type>()) + MCUTimer::counter();
    }
    
    static void isr() {
        ++mCounter;
    }
    
private:
    static volatile ValueType mCounter;
};
template<typename MCUTimer, typename ValueType>
volatile ValueType SoftTimer<MCUTimer, ValueType>::mCounter = 0;