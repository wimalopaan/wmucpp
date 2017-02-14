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

#include <stdint.h>

namespace AVR {

template<uint8_t TimerN, typename MCU = DefaultMcuType>
struct TimerParameter;

template<>
struct TimerParameter<0, ATMega328P> {
    using PortD = AVR::Port<ATMega328P::PortRegister, AVR::D>;
    typedef AVR::Pin<PortD, 6> ocAPin;
    typedef AVR::Pin<PortD, 5> ocBPin;
    typedef AVR::Timer8Bit<0, ATMega328P> timer_type;
    typedef timer_type::value_type value_type;
    
    using ta = AVR::ATMega328P::Timer8Bit::TCCRA;
    static constexpr ta tccra = ta::coma0 | ta::coma1 | ta::comb0 | ta::comb1 | ta::wgm0 | ta::wgm1;
    
    using tb = AVR::ATMega328P::Timer8Bit::TCCRB;
    static constexpr tb tccrb{0};
    
    static constexpr uint8_t top = 0xff;
};
template<>
struct TimerParameter<1, ATMega328P> {
    using PortB = AVR::Port<ATMega328P::PortRegister, AVR::B>;
    typedef AVR::Pin<PortB, 1> ocAPin;
    typedef AVR::Pin<PortB, 2> ocBPin;
    typedef AVR::Timer16Bit<1, ATMega328P> timer_type;
    typedef timer_type::value_type value_type;

    using ta = AVR::ATMega328P::Timer16Bit::TCCRA;
    static constexpr ta tccra = ta::coma0 | ta::coma1 | ta::comb0 | ta::comb1 | ta::wgm0 | ta::wgm1; 
    
    using tb = AVR::ATMega328P::Timer16Bit::TCCRB;
    static constexpr tb tccrb{tb::wgm2}; 

    static constexpr value_type top = 0x3ff;
};
template<>
struct TimerParameter<2, ATMega328P> {
    using PortD = AVR::Port<ATMega328P::PortRegister, AVR::D>;
    using PortB = AVR::Port<ATMega328P::PortRegister, AVR::B>;
    typedef AVR::Pin<PortB, 3> ocAPin;
    typedef AVR::Pin<PortD, 3> ocBPin;
    typedef AVR::Timer8Bit<2, ATMega328P> timer_type;
    typedef timer_type::value_type value_type;
    
    using ta = AVR::ATMega328P::Timer8Bit::TCCRA;
    static constexpr ta tccra = ta::coma0 | ta::coma1 | ta::comb0 | ta::comb1 | ta::wgm0 | ta::wgm1;
    
    using tb = AVR::ATMega328P::Timer8Bit::TCCRB;
    static constexpr tb tccrb{0};
    
    static constexpr uint8_t top = 0xff;
};

}