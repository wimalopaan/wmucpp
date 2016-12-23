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

#include <stdint.h>
#include "mcu/avr/adcomparator.h"
#include "mcu/avr/isr.h"
#include "std/limits.h"
#include "util/bits.h"
#include "util/disable.h"
#include "units/physical.h"

template<uint8_t ADCompNumber, typename MCUTimer>
class RpmFromAnalogComparator final : public IsrBaseHandler<typename AVR::ISR::AdComparator<ADCompNumber>::Edge> {
    RpmFromAnalogComparator() = delete;
public:
    typedef typename MCUTimer::mcu_type mcu_type;
    typedef MCUTimer mcu_timer_type;
    typedef typename MCUTimer::value_type value_type;
    static constexpr auto mcuTimer = MCUTimer::mcuTimer;
    
    static void init() {
        AVR::AdComparator<ADCompNumber>::init();
    }
    
    static value_type period() {
        if constexpr(mcu_type::template is_atomic<value_type>()) {
            return mPeriod;
        }
        else {
            Scoped<DisbaleInterrupt> di;
            return mPeriod;
        }
    }
    
//    static std::hertz frequency() {
////        return mcu_timer_type::frequency();
//    }
    
    static void isr() {
//        mPeriod = (mcuTimer()->tcnt - mTimerStartValue + std::numeric_limits<value_type>::max() + 1) % (std::numeric_limits<value_type>::max() + 1);
//        mTimerStartValue = mcuTimer()->tcnt;
    }
private:
    static volatile value_type mTimerStartValue;
    static volatile value_type mPeriod;
};

template<uint8_t ADCompNumber, typename MCUTimer>
volatile typename RpmFromAnalogComparator<ADCompNumber, MCUTimer>::value_type RpmFromAnalogComparator<ADCompNumber, MCUTimer>::mPeriod = 0;

template<uint8_t ADCompNumber, typename MCUTimer>
volatile typename RpmFromAnalogComparator<ADCompNumber, MCUTimer>::value_type RpmFromAnalogComparator<ADCompNumber, MCUTimer>::mTimerStartValue = 0;
