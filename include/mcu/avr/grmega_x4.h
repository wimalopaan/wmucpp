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
    namespace AD {
        template<AVR::ATMega_X4 MCU>
        struct VRef<V1_1, MCU> {
            static constexpr float value = 1.1;
            static constexpr auto refs = MCU::Adc::MUX::refs1;
        };
        template<AVR::ATMega_X4 MCU>
        struct VRef<V2_56, MCU> {
            static constexpr auto refs = MCU::Adc::MUX::refs1 | MCU::Adc::MUX::refs0;
            static constexpr float value = 2.56;
        };
    }
    
    template<AVR::ATMega_X4 MCU>
    struct TimerParameter<0, MCU> {
        using PortB = AVR::Port<typename MCU::PortRegister, AVR::B>;
        typedef AVR::Pin<PortB, 3> ocAPin;
        typedef AVR::Pin<PortB, 4> ocBPin;
        typedef AVR::Timer8Bit<0, MCU> timer_type;
        using MCUTimer = typename timer_type::mcu_timer_type;
        typedef typename timer_type::value_type value_type;
        static constexpr const auto mcuTimer = getBaseAddr<MCUTimer, 0>;
        
        using ta = typename MCU::Timer8Bit::TCCRA;
        using tb = typename MCU::Timer8Bit::TCCRB;
        
        template<typename Mode = Inverting>
        struct FastPwm1 { // FastPWM 8/10-Bit
            static void setup() {
                mcuTimer()->tccra.template set<tccra>();
                mcuTimer()->tccrb.template add<tccrb, DisbaleInterrupt<NoDisableEnable>>();
                *mcuTimer()->ocra = 0;
                *mcuTimer()->ocrb = 0;
            }
            struct A {
                static void off() {
                    mcuTimer()->tccra.template clear<cha, DisbaleInterrupt<NoDisableEnable>>();
                    ocAPin::high();
                }
                static void on() {
                    mcuTimer()->tccra.template add<cha, DisbaleInterrupt<NoDisableEnable>>();
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
                    mcuTimer()->tccra.template clear<chb, DisbaleInterrupt<NoDisableEnable>>();
                    ocBPin::high();
                }
                static void on() {
                    mcuTimer()->tccra.template add<chb, DisbaleInterrupt<NoDisableEnable>>();
                }
                static void ocr(const typename timer_type::value_type& v) {
                    *mcuTimer()->ocrb = v;
                }
                static value_type ocr() {
                    return *mcuTimer()->ocrb;
                }
            };
            static constexpr ta cha = ta::coma0 | ta::coma1;
            static constexpr ta chb = ta::comb0 | ta::comb1; 
            static constexpr ta tccra = cha | chb | ta::wgm0 | ta::wgm1; 
            static constexpr tb tccrb{0};
            static constexpr uint8_t top = 0xff;
        };
        //    struct FastPwm1 {
        //        static constexpr ta tccra = ta::coma0 | ta::coma1 | ta::comb0 | ta::comb1 | ta::wgm0 | ta::wgm1;
        //        static constexpr tb tccrb{0};
        //        static constexpr uint8_t top = 0xff;
        //    };
    };
    template<AVR::ATMega_X4 MCU>
    struct TimerParameter<1, MCU> {
        using PortD = AVR::Port<typename MCU::PortRegister, AVR::D>;
        typedef AVR::Pin<PortD, 5> ocAPin;
        typedef AVR::Pin<PortD, 4> ocBPin;
        typedef AVR::Timer16Bit<1, MCU> timer_type;
        typedef typename timer_type::value_type value_type;
        
        using ta = typename MCU::Timer16Bit::TCCRA;
        using tb = typename MCU::Timer16Bit::TCCRB;
        struct FastPwm1 { // inverting PWM
            static constexpr ta cha = ta::coma0 | ta::coma1;
            static constexpr ta chb = ta::comb0 | ta::comb1; 
            static constexpr ta tccra = cha | chb | ta::wgm0 | ta::wgm1; 
            static constexpr tb tccrb = tb::wgm2; 
            static constexpr value_type top = 0x3ff;
        };    
        struct FastPwm2 { // FastPWM top=icr
            static constexpr ta cha = ta::coma1;
            static constexpr ta chb = ta::comb1; 
            static constexpr ta tccra = cha | chb | ta::wgm1;
            static constexpr tb tccrb = tb::wgm2 | tb::wgm3; 
        };
    };
    template<AVR::ATMega_X4 MCU>
    struct TimerParameter<2, MCU> {
        using PortD = AVR::Port<typename MCU::PortRegister, AVR::D>;
        typedef AVR::Pin<PortD, 7> ocAPin;
        typedef AVR::Pin<PortD, 6> ocBPin;
        typedef AVR::Timer8Bit<2, MCU> timer_type;
        using MCUTimer = typename timer_type::mcu_timer_type;
        typedef typename timer_type::value_type value_type;
        static constexpr const auto mcuTimer = getBaseAddr<MCUTimer, 2>;
        
        using ta = typename MCU::Timer8Bit::TCCRA;
        using tb = typename MCU::Timer8Bit::TCCRB;

        template<typename Mode = Inverting>
        struct FastPwm1 { // FastPWM 8/10-Bit
            static void setup() {
                mcuTimer()->tccra.template set<tccra>();
                mcuTimer()->tccrb.template add<tccrb, DisbaleInterrupt<NoDisableEnable>>();
                *mcuTimer()->ocra = 0;
                *mcuTimer()->ocrb = 0;
            }
            struct A {
                static void off() {
                    mcuTimer()->tccra.template clear<cha, DisbaleInterrupt<NoDisableEnable>>();
                    ocAPin::high();
                }
                static void on() {
                    mcuTimer()->tccra.template add<cha, DisbaleInterrupt<NoDisableEnable>>();
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
                    mcuTimer()->tccra.template clear<chb, DisbaleInterrupt<NoDisableEnable>>();
                    ocBPin::high();
                }
                static void on() {
                    mcuTimer()->tccra.template add<chb, DisbaleInterrupt<NoDisableEnable>>();
                }
                static void ocr(const typename timer_type::value_type& v) {
                    *mcuTimer()->ocrb = v;
                }
                static value_type ocr() {
                    return *mcuTimer()->ocrb;
                }
            };
            static constexpr ta cha = ta::coma0 | ta::coma1;
            static constexpr ta chb = ta::comb0 | ta::comb1; 
            static constexpr ta tccra = cha | chb | ta::wgm0 | ta::wgm1; 
            static constexpr tb tccrb{0};
            static constexpr uint8_t top = 0xff;
        };
//        struct FastPwm1 {
//            static constexpr ta cha = ta::coma0 | ta::coma1;
//            static constexpr ta chb = ta::comb0 | ta::comb1; 
//            static constexpr ta tccra = cha | chb | ta::wgm0 | ta::wgm1; 
//            static constexpr tb tccrb{}; 
//            static constexpr uint8_t top = 0xff;
//        };
//        struct FastPwm2 { // FastPWM top=oxff
//            static constexpr ta tccra = ta::coma1 | ta::comb1 | ta::wgm1 | ta::wgm0;
//            static constexpr tb tccrb{}; 
//            static constexpr uint8_t top = 0xff;
//        };
    };
    template<AVR::ATMega_X4 MCU>
    struct TimerParameter<3, MCU> {
        using PortB = AVR::Port<typename MCU::PortRegister, AVR::B>;
        typedef AVR::Pin<PortB, 6> ocAPin;
        typedef AVR::Pin<PortB, 7> ocBPin;
        typedef AVR::Timer16Bit<3, MCU> timer_type;
        typedef typename timer_type::value_type value_type;
        
        using ta = typename MCU::Timer16Bit::TCCRA;
        using tb = typename MCU::Timer16Bit::TCCRB;
        
        struct FastPwm1 { // FastPWM 10-Bit
            static constexpr ta cha = ta::coma0 | ta::coma1;
            static constexpr ta chb = ta::comb0 | ta::comb1; 
            static constexpr ta tccra = cha | chb | ta::wgm0 | ta::wgm1; 
            static constexpr tb tccrb = tb::wgm2; 
            static constexpr value_type top = 0x3ff;
        };    
        struct FastPwm2 {  // FastPWM top=icr
            static constexpr ta cha = ta::coma1 ;//| ta::coma0; // inervted
            static constexpr ta chb = ta::comb1 ;//| ta::comb0; 
            static constexpr ta tccra = cha | chb | ta::wgm1;
            static constexpr tb tccrb = tb::wgm2 | tb::wgm3; 
        };
        
        //    static constexpr ta tccra = ta::coma0 | ta::coma1 | ta::comb0 | ta::comb1 | ta::wgm0 | ta::wgm1; 
        //    static constexpr tb tccrb{tb::wgm2}; 
        //    static constexpr value_type top = 0x3ff;
    };
    
    template<AVR::ATMega_X4 MCU>
    struct TimerParameter<4, MCU> {
        using PortB = AVR::Port<typename MCU::PortRegister, AVR::B>;
        using PortC = AVR::Port<typename MCU::PortRegister, AVR::C>;
        typedef AVR::Pin<PortB, 7> ocAPin;
        typedef AVR::Pin<PortC, 4> ocBPin;
        typedef AVR::Timer16Bit<4, MCU> timer_type;
        typedef typename timer_type::value_type value_type;
        
        using ta = typename MCU::Timer16Bit::TCCRA;
        static constexpr ta tccra = ta::coma0 | ta::coma1 | ta::comb0 | ta::comb1 | ta::wgm0 | ta::wgm1; 
        
        using tb = typename MCU::Timer16Bit::TCCRB;
        static constexpr tb tccrb{tb::wgm2}; 
        
        static constexpr value_type top = 0x3ff;
    };
    
}
