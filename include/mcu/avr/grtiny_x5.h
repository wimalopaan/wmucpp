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

#pragma once

#include <cstdint>
#include "mcu/avr8.h"

namespace AVR {
    
    template<AVR::ATTiny_X5 MCU>
    struct TimerParameter<0, MCU> {
        using PortB = AVR::Port<typename MCU::PortRegister, AVR::B>;
        typedef AVR::Pin<PortB, 0> ocAPin; 
        typedef AVR::Pin<PortB, 1> ocBPin;
        typedef AVR::Timer8Bit<0, MCU> timer_type;
        using MCUTimer = typename timer_type::mcu_timer_type;
        static constexpr const auto mcuTimer = getBaseAddr<MCUTimer, 0>;
        typedef typename timer_type::value_type value_type;
        
        using ta = typename MCU::Timer8Bit::TCCRA;
        using tb = typename MCU::Timer8Bit::TCCRB;
        
        template<typename Mode = Inverting>
        struct FastPwm1 { // FastPWM 
            static void setup() {
                mcuTimer()->tccra.template set<tccra>();
                mcuTimer()->tccrb.template add<DisbaleInterrupt<NoDisableEnable>>();
                *mcuTimer()->ocra = 0;
                *mcuTimer()->ocrb = 0;
            }
            static constexpr ta cha = ta::coma0 | ta::coma1;
            static constexpr ta chb = ta::comb0 | ta::comb1; 
            static constexpr ta tccra = cha | chb | ta::wgm0 | ta::wgm1; 
            static constexpr tb tccrb{0};
            static constexpr uint8_t top = 0xff;
        };
    };

    template<AVR::ATTiny_X5 MCU>
    struct TimerParameter<1, MCU> {
        using PortB = AVR::Port<typename MCU::PortRegister, AVR::B>;
        typedef AVR::Pin<PortB, 1> ocAPin;
        typedef AVR::Pin<PortB, 4> ocBPin;
        typedef AVR::Timer8Bit<1, MCU> timer_type;
        using MCUTimer = typename timer_type::mcu_timer_type;
        static constexpr const auto mcuTimer = getBaseAddr<MCUTimer, 1>;
        typedef typename timer_type::value_type value_type;
        
        using tt = typename MCU::Timer8BitHighSpeed::TCCR;
        using tg = typename MCU::Timer8BitHighSpeed::GT;
        
        template<typename Mode = Inverting>
        struct FastPwm1 { // FastPWM 
            static constexpr uint8_t top = 0xff;
            static void setup() {
                mcuTimer()->tccr.template add<cha>();
                mcuTimer()->gtccr.template add<chb>();
                *mcuTimer()->ocra = 0;
                *mcuTimer()->ocrb = 0;
            }
            struct A {
                static void off() {
                    mcuTimer()->tccr.template clear<cha, DisbaleInterrupt<NoDisableEnable>>();
                    ocAPin::high();
                }
                static void on() {
                    mcuTimer()->tccr.template add<cha, DisbaleInterrupt<NoDisableEnable>>();
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
                    mcuTimer()->gtccr.template clear<chb, DisbaleInterrupt<NoDisableEnable>>();
                    ocBPin::high();
                }
                static void on() {
                    mcuTimer()->gtccr.template add<chb, DisbaleInterrupt<NoDisableEnable>>();
                }
                static void ocr(const typename timer_type::value_type& v) {
                    *mcuTimer()->ocrb = v;
                }
                static value_type ocr() {
                    return *mcuTimer()->ocrb;
                }
            };
        };
        template<typename Mode = Inverting>
        struct FastPwmA { // FastPWM 
            static void setup() {
                mcuTimer()->tccr.template add<cha>();
//                    mcuTimer()->gtccr.template add<chb>();
                *mcuTimer()->ocra = 0;
//                    *mcuTimer()->ocrb = 0;
            }
            struct A {
                static void off() {
                    mcuTimer()->tccr.template clear<cha, DisbaleInterrupt<NoDisableEnable>>();
                    ocAPin::high();
                }
                static void on() {
                    mcuTimer()->tccr.template add<cha, DisbaleInterrupt<NoDisableEnable>>();
                }
                static void ocr(const typename timer_type::value_type& v) {
                    *mcuTimer()->ocra = v;
                }
                static value_type ocr() {
                    return *mcuTimer()->ocra;
                }
            };
        };
        static constexpr tt cha = tt::coma0 | tt::coma1 | tt::pwma;
        static constexpr tg chb = tg::comb0 | tg::comb1 | tg::pwmb; 
        static constexpr uint8_t top = 0xff;
    };
    
}
