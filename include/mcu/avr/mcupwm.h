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

#include "mcu/avr/mcutimer.h"
#include "mcu/ports.h"
#include "mcu/register.h"
#include "mcu/avr/groups.h"
#include "units/percent.h"
#include "std/limits"

namespace AVR {
    
    template<uint8_t TimerN, typename MCU = DefaultMcuType>
    class PWM {
        typedef MCU mcu_type;
        typedef typename AVR::TimerParameter<TimerN, MCU>::timer_type timer_type;
        using MCUTimer = typename timer_type::mcu_timer_type;
        typedef typename MCUTimer::value_type value_type;
        static constexpr const auto mcuTimer = getBaseAddr<MCUTimer, TimerN>;
        using pwmA = typename AVR::TimerParameter<TimerN, MCU>::ocAPin;
        using pwmB = typename AVR::TimerParameter<TimerN, MCU>::ocBPin;
    public:
        struct A {
            static void off() {
                mcuTimer()->tccra.template clear<AVR::TimerParameter<TimerN, MCU>::FastPwm1::cha>();
                pwmA::high();
            }
            static void on() {
                mcuTimer()->tccra.template add<AVR::TimerParameter<TimerN, MCU>::FastPwm1::cha>();
            }
            static void ocr(const typename timer_type::value_type& v) {
                *mcuTimer()->ocra = v;
            }
            static value_type ocr() {
                return *mcuTimer()->ocra;
            }
        };
        struct B {
            static void off() {
                mcuTimer()->tccra.template clear<AVR::TimerParameter<TimerN, MCU>::FastPwm1::chb>();
                pwmB::high();
            }
            static void on() {
                mcuTimer()->tccra.template add<AVR::TimerParameter<TimerN, MCU>::FastPwm1::chb>();
            }
            static void ocr(const typename timer_type::value_type& v) {
                *mcuTimer()->ocrb = v;
            }
            static value_type ocr() {
                return *mcuTimer()->ocrb;
            }
        };
        
        template<const std::hertz& MinFrequency>
        static void init() {
            pwmA::template dir<AVR::Output>();
            pwmB::template dir<AVR::Output>();
            
            constexpr auto prescaler = AVR::Util::prescalerForAbove<timer_type>(MinFrequency);
            static_assert(prescaler > 0, "wrong prescaler");
            timer_type::template prescale<prescaler>();
            mcuTimer()->tccra.template set<AVR::TimerParameter<TimerN, MCU>::FastPwm1::tccra>();
            mcuTimer()->tccrb.template add<AVR::TimerParameter<TimerN, MCU>::FastPwm1::tccrb>();
            *mcuTimer()->ocra = 0;
            *mcuTimer()->ocrb = 0;
        }
        template<typename Channel>
        static void pwm(const std::percent& p) {
            if (p > std::percent{0}) {
                typename timer_type::value_type v = std::expand(p, typename timer_type::value_type(0), AVR::TimerParameter<TimerN, MCU>::FastPwm1::top);
                Channel::on();
                Channel::ocr(v);
            }
            else {
                Channel::off();
            }
        }
        template<typename Channel>
        static value_type ocr() {
            return Channel::ocr();
        }
        static std::hertz frequency() {
            return timer_type::frequency();
        }
    };
    
}
