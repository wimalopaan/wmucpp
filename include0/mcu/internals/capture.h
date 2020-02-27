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
#include "../common/concepts.h"

namespace AVR {
    using namespace AVR::Util::Timer;
    using namespace External::Units;
    
    template<typename, typename MCU = DefaultMcuType>
    struct Capture;

    template<AVR::Concepts::ComponentNumber CNumber, AVR::Concepts::At01Series MCU>
    struct Capture<CNumber, MCU> final {
        inline static constexpr uint8_t number = CNumber::value;
        using value_type = uint16_t;
        
        static inline constexpr auto mcu_tcb = AVR::getBaseAddr<typename MCU::TCB, number>;
        
        using ctrla_t = MCU::TCB::CtrlA_t;
        using ctrlb_t = MCU::TCB::CtrlB_t;
        using intflags_t = MCU::TCB::IntFlags_t;
        
        inline static constexpr auto on_flags  = ctrla_t::clkdiv1 | ctrla_t::enable;
        inline static constexpr auto off_flags = ctrla_t::clkdiv1;
        
        inline static void init() {
            mcu_tcb()->ctrlb.template set<ctrlb_t::mode_capt>();
            mcu_tcb()->ctrla.template set<on_flags>();
            mcu_tcb()->intflags.template reset<intflags_t::capt>();
        }
        
        inline static void off() {
            mcu_tcb()->ctrla.template set<off_flags>();
        }

        inline static void on() {
            mcu_tcb()->intflags.template reset<intflags_t::capt>();
            *mcu_tcb()->cnt = 0;
            mcu_tcb()->ctrla.template set<on_flags>();
        }
        
        inline static value_type value() {
            return *mcu_tcb()->ccmp;
        }
    };
    
    template<AVR::Concepts::ComponentNumber CNumber, AVR::Concepts::AtMega_X MCU>
    struct Capture<CNumber, MCU> : TimerBase16Bit<CNumber::value, MCU>{
       
        using base = TimerBase16Bit<CNumber::value, MCU>;
        
        using flags_type = mcu_timer_interrupts_flags_t<base::number>;
        
        Capture() = delete;
        
        template<int PreScale>
        static inline void prescale() {
            constexpr auto bits = bitsFrom<PreScale>(prescaler_bits_v<base::number>);
            static_assert(isset(bits), "wrong prescaler");
            base::mcu_timer()->tccrb.template set<bits>();
        }
        
        static inline void off() {
            constexpr auto bits = bitsFrom<0>(prescaler_bits_v<base::number>);
            base::mcu_timer()->tccrb.template set<bits>();
            *base::mcu_timer()->tcnt = 0;
        }
        
        
        inline static void ocra(base::value_type v) {
            *base::mcu_timer()->ocra = v;
        }
        
        static inline base::value_type  counter() {
            return *base::mcu_timer()->tcnt;
        }
        static inline base::value_type icr() {
            return *base::mcu_timer()->icr;
        }
        
        static inline void noiseCancel(bool on = true) {
            if (on) {
                base::mcu_timer()->tccrb.template add<base::tb::icnc>();
            }    
            else {
                base::mcu_timer()->tccrb.template clear<base::tb::icnc>();
            }
        }
        
        static inline void captureRising(bool on = true) {
            if (on) {
                base::mcu_timer()->tccrb.template clear<base::tb::ices>();
            }    
            else {
                base::mcu_timer()->tccrb.template add<base::tb::ices>();
            }
        }
        
        inline static void reset() {
            *base::mcu_timer()->tcnt = 0;
        }
        
        template<mcu_timer_interrupts_flags_t<base::number> Flag>
        inline static void clearFlag() {
            base::mcu_timer_interrupts()->tifr.template reset<Flag>();
        }
        
        
        template<mcu_timer_interrupts_flags_t<base::number> Compare, etl::Concepts::Callable Callable>
        inline static void periodic(const Callable& f) {
            if (base::mcu_timer_interrupts()->tifr.template isSet<Compare>()) {
                f();
                base::mcu_timer_interrupts()->tifr.template reset<Compare>();
            } 
        }
        
        static inline void init() {
            base::mcu_timer()->tccrb.template add<base::tb::icnc | base::tb::ices, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
        }
    };
    
    
}
