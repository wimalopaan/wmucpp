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

#include "mcu/ports.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/groups.h"
#include "mcu/avr/ppmbase.h"
#include "mcu/avr/util.h"
#include "units/percent.h"
#include "util/disable.h"
#include "util/rational.h"

namespace AVR {
    
    template<uint8_t TimerN, typename Prescaler = void, typename MCU = DefaultMcuType>
    class PPM final {
        PPM() = delete;
    public:
        typedef MCU mcu_type;
        typedef typename AVR::TimerParameter<TimerN, MCU>::timer_type timer_type;
        using MCUTimer = typename timer_type::mcu_timer_type;
        using ocAPin = typename AVR::TimerParameter<TimerN, MCU>::ocAPin;
        using ocBPin = typename AVR::TimerParameter<TimerN, MCU>::ocBPin;
        static constexpr const auto mcuTimer = AVR::getBaseAddr<MCUTimer, TimerN>;
        
        using parameter = PPMParameter<typename AVR::TimerParameter<TimerN, MCU>::timer_type, Prescaler>;
        
        typedef uint_ranged<uint16_t, parameter::ocMin, parameter::ocMax> ranged_type;
        
        static void init() {
            ocAPin::template dir<AVR::Output>();
            ocBPin::template dir<AVR::Output>();
            
            timer_type::template prescale<parameter::prescaler>();
            
            *mcuTimer()->icr = parameter::ocFrame;
            
            if constexpr(std::is_same<Prescaler, void>::value) {
                mcuTimer()->tccra.template set<AVR::TimerParameter<TimerN, MCU>::FastPwm2::tccra>();
                mcuTimer()->tccrb.template add<AVR::TimerParameter<TimerN, MCU>::FastPwm2::tccrb, DisbaleInterrupt<NoDisableEnable>>();
            }
            else {
                mcuTimer()->tccra.template set<AVR::TimerParameter<TimerN, MCU>::FastPwm1::tccra>();
                mcuTimer()->tccrb.template add<AVR::TimerParameter<TimerN, MCU>::FastPwm1::tccrb, DisbaleInterrupt<NoDisableEnable>>();
            }
            constexpr typename timer_type::value_type medium = (parameter::ocMin + parameter::ocMax) / 2;
            *mcuTimer()->ocra = medium;
            *mcuTimer()->ocrb = medium;
        }
        
        struct A {
            static void off() {
                mcuTimer()->tccra.template clear<AVR::TimerParameter<TimerN, MCU>::FastPwm2::cha, DisbaleInterrupt<NoDisableEnable>>();
                ocAPin::low();
            }
            static void on() {
                mcuTimer()->tccra.template add<AVR::TimerParameter<TimerN, MCU>::FastPwm2::cha, DisbaleInterrupt<NoDisableEnable>>();
            }
            static void ocr(const typename timer_type::value_type& v) {
                *mcuTimer()->ocra = v;
            }
        };
        struct B {
            static void off() {
                mcuTimer()->tccra.template clear<AVR::TimerParameter<TimerN, MCU>::FastPwm2::chb, DisbaleInterrupt<NoDisableEnable>>();
                ocBPin::low();
            }
            static void on() {
                mcuTimer()->tccra.template add<AVR::TimerParameter<TimerN, MCU>::FastPwm2::chb, DisbaleInterrupt<NoDisableEnable>>();
            }
            static void ocr(const typename timer_type::value_type& v) {
                *mcuTimer()->ocrb = v;
            }
        };
        
        template<typename Channel>
        static void ppm(const std::percent& width) {
            uint16_t ocr = std::expand(width, (uint32_t)parameter::ocMin, (uint32_t)parameter::ocMax);
            Channel::ocr(ocr);
        }
        template<typename Channel, typename T, T L, T U>
        static void ppm(uint_ranged<T, L, U> raw) {
            T v1 = raw.toInt() - L;
            constexpr uint64_t denom = U - L;
            constexpr uint64_t nom = ranged_type::Upper - ranged_type::Lower;
            uint16_t ocr = ::Util::RationalDivider<uint16_t, nom, denom>::scale(v1) + ranged_type::Lower;
            Channel::ocr(ocr);
        }
        template<typename Channel, typename T, T L, T U>
        static void ppm(uint_ranged_NaN<T, L, U> raw) {
            if (raw) {
                T v1 = raw.toInt() - L;
                constexpr uint64_t denom = U - L;
                constexpr uint64_t nom = ranged_type::Upper - ranged_type::Lower;
                uint16_t ocr = ::Util::RationalDivider<uint16_t, nom, denom>::scale(v1) + ranged_type::Lower;
                Channel::ocr(ocr);
            }
        }
    };
    
}
