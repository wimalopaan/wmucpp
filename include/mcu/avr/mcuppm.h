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

#pragma once

#include "mcu/ports.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/groups.h"
#include "mcu/avr/ppmbase.h"
#include "mcu/avr/util.h"
#include "units/percent.h"
#include "util/disable.h"

namespace AVR {

template<uint8_t TimerN, typename MCU = DefaultMcuType>
class PPM final : PPMBase<typename AVR::TimerParameter<TimerN, MCU>::timer_type> {
    PPM() = delete;
public:
    typedef MCU mcu_type;
    typedef typename AVR::TimerParameter<TimerN, MCU>::timer_type timer_type;
    using MCUTimer = typename timer_type::mcu_timer_type;
    static constexpr const auto mcuTimer = AVR::getBaseAddr<MCUTimer, TimerN>;
    using ocAPin = typename AVR::TimerParameter<TimerN, MCU>::ocAPin;
    using ocBPin = typename AVR::TimerParameter<TimerN, MCU>::ocAPin;

    using PPMBase<timer_type>::ocMin;
    using PPMBase<timer_type>::ocMax;
    using PPMBase<timer_type>::ocFrame;
    using PPMBase<timer_type>::prescaler;
    using PPMBase<timer_type>::mcuInterrupts;
    
    static void init() {
        ocAPin::template dir<AVR::Output>();
        ocBPin::template dir<AVR::Output>();

        timer_type::template prescale<prescaler>();
        
        *mcuTimer()->icr = ocFrame;
        mcuTimer()->tccra.set(AVR::TimerParameter<TimerN, MCU>::FastPwm2::tccra);
        mcuTimer()->tccrb.set(AVR::TimerParameter<TimerN, MCU>::FastPwm2::tccrb);
    }

    struct A {
        static void ocr(const typename timer_type::value_type& v) {
            *mcuTimer()->ocra = v;
        }
    };
    struct B {
        static void ocr(const typename timer_type::value_type& v) {
            *mcuTimer()->ocrb = v;
        }
    };
    template<typename Channel>
    static void ppm(const std::percent& width) {
        uint16_t ocr = std::expand(width, ocMin, ocMax);
        Channel::ocr(ocr);
    }
    
private:
};


}
