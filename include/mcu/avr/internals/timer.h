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

#include "mcu/avr/util.h"

namespace AVR::Internals {
    namespace Timers {
        enum class Purpose {General, Ppm, PWM, Undefined};
        
        template<typename MCU>
        struct Timer_8Bit_A {
            
        };
        template<typename MCU>
        struct Timer_8Bit_B {
            
        };
        template<typename MCU>
        struct Timer_16Bit_A {
            
        };
    }
    namespace detail {
        template<uint8_t N, typename MCU>
        struct TimerType;
        
        template<>
        struct TimerType<1, AVR::ATMega328PB> {
            using type = Timers::Timer_16Bit_A<AVR::ATMega328PB>;
            using mcu = AVR::ATMega328PB::Timer16Bit;
            
            
//            using ocAPin = 
            
        };
        
        template<uint8_t N, typename MCU>
        using timer_type = typename TimerType<N, MCU>::type;

        template<uint8_t N, typename MCU>
        using mcutimer_type = typename TimerType<N, MCU>::mcu;
    }
    
    template<uint8_t N, typename MCU = DefaultMcuType>
    struct Timer {
        using timer_type = detail::timer_type<N, MCU>;
        using mcutimer_type = detail::mcutimer_type<N, MCU>;
        
        template<AVR::Internals::Timers::Purpose P>
        static constexpr inline void setup() {
            if constexpr(P == Timers::Purpose::Ppm) {
//                AVR::Util::calculatePpmOutParameter<mcutimer_type, uint16_t>();                
            }   
            else {
                static_assert(std::false_v<P>, "wrong timer purpose");
            }
        }
//        static constexpr inline void setup(const std::hertz& f) {
//            constexpr auto tsd = AVR::Util::calculate<MCU>(f);
//        }
    };
}
