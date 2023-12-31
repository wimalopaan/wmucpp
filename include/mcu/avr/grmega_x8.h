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
        template<AVR::ATMega_X8 MCU>
        struct VRef<V1_1, MCU> {
            static constexpr auto refs = MCU::Adc::MUX::refs1 | MCU::Adc::MUX::refs0;
            static constexpr float value = 1.1;
        };
    }
    
    template<AVR::ATMega_X8 MCU>
    struct TimerParameter<0, MCU> final {
        using PortD = AVR::Port<typename MCU::PortRegister, AVR::D>;
        typedef AVR::Pin<PortD, 6> ocAPin;
        typedef AVR::Pin<PortD, 5> ocBPin;
        typedef AVR::Timer8Bit<0, MCU> timer_type;
        using MCUTimer = typename timer_type::mcu_timer_type;
        static constexpr const auto mcuTimer = getBaseAddr<MCUTimer, 0>;
        typedef typename timer_type::value_type value_type;
        
        using ta = typename MCU::Timer8Bit::TCCRA;
        using tb = typename MCU::Timer8Bit::TCCRB;
        
        template<typename Mode = Inverting>
        struct FastPwm1 {
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
        
    };
    template<AVR::ATMega_X8 MCU>
    struct TimerParameter<1, MCU> {
        using PortB = AVR::Port<typename MCU::PortRegister, AVR::B>;
        typedef AVR::Pin<PortB, 1> ocAPin;
        typedef AVR::Pin<PortB, 2> ocBPin;
        typedef AVR::Timer16Bit<1, MCU> timer_type;
        using MCUTimer = typename timer_type::mcu_timer_type;
        static constexpr const auto mcuTimer = getBaseAddr<MCUTimer, 1>;
        typedef typename timer_type::value_type value_type;
        
        using ta = typename MCU::Timer16Bit::TCCRA;
        using tb = typename MCU::Timer16Bit::TCCRB;
        struct FastPwm1 {
            static constexpr ta cha = ta::coma0 | ta::coma1;
            static constexpr ta chb = ta::comb0 | ta::comb1; 
            static constexpr ta tccra = cha | chb | ta::wgm0 | ta::wgm1; 
            static constexpr tb tccrb{tb::wgm2}; 
            static constexpr value_type top = 0x3ff;

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
        };
        struct FastPwm2 { // FastPWM top=icr
            static constexpr ta cha = ta::coma1;
            static constexpr ta chb = ta::comb1; 
            static constexpr ta tccra = cha | chb | ta::wgm1;
            static constexpr tb tccrb = tb::wgm2 | tb::wgm3; 
        };
    };
    template<AVR::ATMega_X8 MCU>
    struct TimerParameter<2, MCU> {
        using PortD = AVR::Port<typename MCU::PortRegister, AVR::D>;
        using PortB = AVR::Port<typename MCU::PortRegister, AVR::B>;
        typedef AVR::Pin<PortB, 3> ocAPin;
        typedef void ocBPin;
//        typedef AVR::Pin<PortD, 3> ocBPin;
        typedef AVR::Timer8Bit<2, MCU> timer_type;
        using MCUTimer = typename timer_type::mcu_timer_type;
        static constexpr const auto mcuTimer = getBaseAddr<MCUTimer, 2>;
        typedef typename timer_type::value_type value_type;
        
        using ta = typename MCU::Timer8Bit::TCCRA;
        using tb = typename MCU::Timer8Bit::TCCRB;
        
        template<typename Mode = Inverting>
        struct FastPwm1 {
            static void setup() {
                mcuTimer()->tccra.template set<tccra>();
                mcuTimer()->tccrb.template add<tccrb, DisbaleInterrupt<NoDisableEnable>>();
                *mcuTimer()->ocra = 0;
                *mcuTimer()->ocrb = 0;
            }
            struct A {
                static void off() {
                    mcuTimer()->tccra.template clear<cha, DisbaleInterrupt<NoDisableEnable>>();
                    if constexpr(std::is_same_v<Mode, Inverting>) {
                        ocAPin::high();
                    }
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
            using B = void;
//            struct B {
//                static void off() {
//                    mcuTimer()->tccra.template clear<chb, DisbaleInterrupt<NoDisableEnable>>();
//                    ocBPin::high();
//                }
//                static void on() {
//                    mcuTimer()->tccra.template add<chb, DisbaleInterrupt<NoDisableEnable>>();
//                }
//                static void ocr(const typename timer_type::value_type& v) {
//                    *mcuTimer()->ocrb = v;
//                }
//                static value_type ocr() {
//                    return *mcuTimer()->ocrb;
//                }
//            };
            static constexpr ta cha = [](){
                if constexpr(std::is_same_v<Mode, Inverting>) {
                    return ta::coma0 | ta::coma1;
                }
                else {
                    return ta::coma1;
                }
            }();
//            static constexpr ta chb = ta::comb0 | ta::comb1; 
            static constexpr ta tccra = cha | ta::wgm0 | ta::wgm1; 
//            static constexpr ta tccra = cha | chb | ta::wgm0 | ta::wgm1; 
            static constexpr tb tccrb{0};
            static constexpr uint8_t top = 0xff;
        };
    };
    template<AVR::ATMega_X8 MCU>
    struct TimerParameter<3, MCU> {
        using PortD = AVR::Port<typename MCU::PortRegister, AVR::D>;
//        typedef AVR::Pin<PortD, 0> ocAPin;
        typedef void ocAPin;
        typedef AVR::Pin<PortD, 2> ocBPin;
//        typedef void ocBPin;
        typedef AVR::Timer16Bit<3, MCU> timer_type;
        using MCUTimer = typename timer_type::mcu_timer_type;
        static constexpr const auto mcuTimer = getBaseAddr<MCUTimer, 3>;
//        typedef typename timer_type::value_type value_type;
        typedef  uint8_t value_type;
        
        using ta = typename MCU::Timer16Bit::TCCRA;
        using tb = typename MCU::Timer16Bit::TCCRB;
        
        template<typename Mode = Inverting>
        struct FastPwm1 {
//            static constexpr ta cha = ta::coma0 | ta::coma1;
            static constexpr ta chb = [](){
                if constexpr(std::is_same_v<Mode, Inverting>) {
                    return ta::comb0 | ta::comb1;
                }
                else {
                    return ta::comb1;
                }
            }();
            
            static constexpr ta tccra = chb | ta::wgm0; 
//            static constexpr ta tccra = cha | chb | ta::wgm0; 
//            static constexpr ta tccra = cha | ta::wgm0 | ta::wgm1; 
//            static constexpr tb tccrb{tb::wgm2}; 
            static constexpr tb tccrb{}; 
//            static constexpr value_type top = 0x3ff;
            static constexpr value_type top = 0xff;

            static void setup() {
                mcuTimer()->tccra.template set<tccra>();
                mcuTimer()->tccrb.template add<tccrb, DisbaleInterrupt<NoDisableEnable>>();
                *mcuTimer()->ocra = 0;
                *mcuTimer()->ocrb = 0;
            }
            
            using A = void;
//            struct A {
//                static void off() {
//                    mcuTimer()->tccra.template clear<cha, DisbaleInterrupt<NoDisableEnable>>();
//                    ocAPin::high();
//                }
//                static void on() {
//                    mcuTimer()->tccra.template add<cha, DisbaleInterrupt<NoDisableEnable>>();
//                }
//                static void ocr(const typename timer_type::value_type& v) {
//                    *mcuTimer()->ocra = v;
//                }
//                static value_type ocr() {
//                    return *mcuTimer()->ocra;
//                }
//            };
//            using B = void;
            struct B {
                static void off() {
                    mcuTimer()->tccra.template clear<chb, DisbaleInterrupt<NoDisableEnable>>();
                    if constexpr(std::is_same_v<Mode, Inverting>) {
                        ocBPin::high();
                    }
                    else {
                        ocBPin::low();
                    }
                }
                static void on() {
                    mcuTimer()->tccra.template add<chb, DisbaleInterrupt<NoDisableEnable>>();
                    ocBPin::high(); // OCM Bug
                }
                static void ocr(const typename timer_type::value_type& v) {
                    *mcuTimer()->ocrb = v;
                }
                static value_type ocr() {
                    return *mcuTimer()->ocrb;
                }
            };
        };
//        struct FastPwm1 {
//            static constexpr ta tccra = ta::coma0 | ta::coma1 | ta::comb0 | ta::comb1 | ta::wgm0 | ta::wgm1; 
//            static constexpr tb tccrb{tb::wgm2}; 
//            static constexpr value_type top = 0x3ff;
//        };
    };
    template<AVR::ATMega_X8 MCU>
    struct TimerParameter<4, MCU> {
        using PortD = AVR::Port<typename MCU::PortRegister, AVR::D>;
        typedef AVR::Pin<PortD, 1> ocAPin;
        typedef AVR::Pin<PortD, 2> ocBPin;
        typedef AVR::Timer16Bit<4, MCU> timer_type;
        typedef typename timer_type::value_type value_type;
        
        using ta = typename MCU::Timer16Bit::TCCRA;
        using tb = typename MCU::Timer16Bit::TCCRB;
        
        struct FastPwm1 {
            static constexpr ta tccra = ta::coma0 | ta::coma1 | ta::comb0 | ta::comb1 | ta::wgm0 | ta::wgm1; 
            static constexpr tb tccrb{tb::wgm2}; 
            static constexpr value_type top = 0x3ff;
        };
    };
    
}
