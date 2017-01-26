/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include "units/percent.h"
#include "std/limits.h"

namespace AVR {

template<uint8_t TimerN, typename MCU = DefaultMcuType>
struct PwmParamter;

template<>
struct PwmParamter<0, ATMega1284P> {
    using PortB = AVR::Port<ATMega1284P::PortRegister, AVR::B>;
    typedef AVR::Pin<PortB, 3> pwmA;
    typedef AVR::Pin<PortB, 4> pwmB;
    typedef AVR::Timer8Bit<0, ATMega1284P> timer_type;
    typedef timer_type::value_type value_type;
    
    // todo: test and cleanup
    
//    static constexpr uint8_t tccra = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) | _BV(COM0B0) // non-inverting mode
//                                     | _BV(WGM00) | _BV(WGM01) ; // 8-bit fast PWM
    
    using ta = AVR::ATMega1284P::Timer8Bit::TCCRA;
    static constexpr ta tccra = ta::coma0 | ta::coma1 | ta::comb0 | ta::comb1 | ta::wgm0 | ta::wgm1;
    
    using tb = AVR::ATMega1284P::Timer8Bit::TCCRB;
    static constexpr tb tccrb{0};
    
//    static constexpr uint8_t tccrb = 0;
    static constexpr uint8_t top = 0xff;
};
template<>
struct PwmParamter<1, ATMega1284P> {
    using PortD = AVR::Port<ATMega1284P::PortRegister, AVR::D>;
    typedef AVR::Pin<PortD, 5> pwmA;
    typedef AVR::Pin<PortD, 4> pwmB;
    typedef AVR::Timer16Bit<1, ATMega1284P> timer_type;
    typedef timer_type::value_type value_type;
//    static constexpr uint8_t tccra = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0) // non-inverting mode
//                                     | _BV(WGM10) | _BV(WGM11) ; // 10-bit fast PWM

    using ta = AVR::ATMega1284P::Timer16Bit::TCCRA;
    static constexpr ta tccra = ta::coma0 | ta::coma1 | ta::comb0 | ta::comb1 | ta::wgm0 | ta::wgm1; 
    
    using tb = AVR::ATMega1284P::Timer16Bit::TCCRB;
    static constexpr tb tccrb{tb::wgm2}; 
//    static constexpr uint8_t tccrb = _BV(WGM12); // 10-bit fast PWM
    static constexpr value_type top = 0x3ff;
};
template<>
struct PwmParamter<2, ATMega1284P> {
    using PortD = AVR::Port<ATMega1284P::PortRegister, AVR::D>;
    typedef AVR::Pin<PortD, 7> pwmA;
    typedef AVR::Pin<PortD, 6> pwmB;
    typedef AVR::Timer8Bit<2, ATMega1284P> timer_type;
    typedef timer_type::value_type value_type;
//    static constexpr uint8_t tccra = _BV(COM2A1) | _BV(COM2A0) | _BV(COM2B1) | _BV(COM2B0) // non-inverting mode
//                                     | _BV(WGM20) | _BV(WGM21) ; // 8-bit fast PWM
    
    using ta = AVR::ATMega1284P::Timer8Bit::TCCRA;
    static constexpr ta tccra = ta::coma0 | ta::coma1 | ta::comb0 | ta::comb1 | ta::wgm0 | ta::wgm1; 

    using tb = AVR::ATMega1284P::Timer8Bit::TCCRB;
    static constexpr tb tccrb{}; 
//    static constexpr uint8_t tccrb = 0;
    static constexpr uint8_t top = 0xff;
};
template<>
struct PwmParamter<3, ATMega1284P> {
    using PortB = AVR::Port<ATMega1284P::PortRegister, AVR::B>;
    typedef AVR::Pin<PortB, 6> pwmA;
    typedef AVR::Pin<PortB, 7> pwmB;
    typedef AVR::Timer16Bit<3, ATMega1284P> timer_type;
    typedef timer_type::value_type value_type;
//    static constexpr uint8_t tccra = _BV(COM3A1) | _BV(COM3A0) | _BV(COM3B1) | _BV(COM3B0) // non-inverting mode
//                                     | _BV(WGM30) | _BV(WGM31) ; // 10-bit fast PWM
    
    using ta = AVR::ATMega1284P::Timer16Bit::TCCRA;
    static constexpr ta tccra = ta::coma0 | ta::coma1 | ta::comb0 | ta::comb1 | ta::wgm0 | ta::wgm1; 

    using tb = AVR::ATMega1284P::Timer16Bit::TCCRB;
    static constexpr tb tccrb{tb::wgm2}; 
//    static constexpr uint8_t tccrb = _BV(WGM32); // 10-bit fast PWM
    static constexpr value_type top = 0x3ff;
};

template<uint8_t TimerN, typename MCU = DefaultMcuType>
class PWM {
    typedef MCU mcu_type;
    typedef typename PwmParamter<TimerN, MCU>::timer_type timer_type;
    using MCUTimer = typename timer_type::mcu_timer_type;
    static constexpr const auto mcuTimer = getBaseAddr<MCUTimer, TimerN>;
    using pwmA = typename PwmParamter<TimerN, MCU>::pwmA;
    using pwmB = typename PwmParamter<TimerN, MCU>::pwmB;
public:
    struct A {
        static void ocr(const typename timer_type::value_type& v) {
            mcuTimer()->ocra = v;
        }
    };
    struct B {
        static void ocr(const typename timer_type::value_type& v) {
            mcuTimer()->ocrb = v;
        }
    };

    static void init() {
        pwmA::template dir<AVR::Output>();
        pwmB::template dir<AVR::Output>();
        timer_type::template prescale<256>();
//        mcuTimer()->tccra |= PwmParamter<TimerN, MCU>::tccra;
        mcuTimer()->tccra.set(PwmParamter<TimerN, MCU>::tccra);
        mcuTimer()->tccrb.set(PwmParamter<TimerN, MCU>::tccrb);
    }
    template<typename Channel>
    static void pwm(const std::percent& p) {
        typename timer_type::value_type v = std::expand(p, typename timer_type::value_type(0), PwmParamter<TimerN, MCU>::top);
        Channel::ocr(v);
    }
};


}
