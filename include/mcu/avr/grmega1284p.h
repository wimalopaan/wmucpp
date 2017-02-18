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
struct TimerParameter<0, ATMega1284P> {
    using PortB = AVR::Port<ATMega1284P::PortRegister, AVR::B>;
    typedef AVR::Pin<PortB, 3> ocAPin;
    typedef AVR::Pin<PortB, 4> ocBPin;
    typedef AVR::Timer8Bit<0, ATMega1284P> timer_type;
    typedef timer_type::value_type value_type;
    
    using ta = AVR::ATMega1284P::Timer8Bit::TCCRA;
    using tb = AVR::ATMega1284P::Timer8Bit::TCCRB;
    struct FastPwm1 {
        static constexpr ta tccra = ta::coma0 | ta::coma1 | ta::comb0 | ta::comb1 | ta::wgm0 | ta::wgm1;
        static constexpr tb tccrb{0};
        static constexpr uint8_t top = 0xff;
    };
};
template<>
struct TimerParameter<1, ATMega1284P> {
    using PortD = AVR::Port<ATMega1284P::PortRegister, AVR::D>;
    typedef AVR::Pin<PortD, 5> ocAPin;
    typedef AVR::Pin<PortD, 4> ocBPin;
    typedef AVR::Timer16Bit<1, ATMega1284P> timer_type;
    typedef timer_type::value_type value_type;

    using ta = AVR::ATMega1284P::Timer16Bit::TCCRA;
    using tb = AVR::ATMega1284P::Timer16Bit::TCCRB;
    struct FastPwm1 {
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
template<>
struct TimerParameter<2, ATMega1284P> {
    using PortD = AVR::Port<ATMega1284P::PortRegister, AVR::D>;
    typedef AVR::Pin<PortD, 7> ocAPin;
    typedef AVR::Pin<PortD, 6> ocBPin;
    typedef AVR::Timer8Bit<2, ATMega1284P> timer_type;
    typedef timer_type::value_type value_type;
    
    using ta = AVR::ATMega1284P::Timer8Bit::TCCRA;
    using tb = AVR::ATMega1284P::Timer8Bit::TCCRB;
    struct FastPwm1 {
        static constexpr ta cha = ta::coma0 | ta::coma1;
        static constexpr ta chb = ta::comb0 | ta::comb1; 
        static constexpr ta tccra = cha | chb | ta::wgm0 | ta::wgm1; 
        static constexpr tb tccrb{}; 
        static constexpr uint8_t top = 0xff;
    };
    struct FastPwm2 { // FastPWM top=oxff
        static constexpr ta tccra = ta::coma1 | ta::comb1 | ta::wgm1 | ta::wgm0;
        static constexpr tb tccrb{}; 
        static constexpr uint8_t top = 0xff;
    };
};
template<>
struct TimerParameter<3, ATMega1284P> {
    using PortB = AVR::Port<ATMega1284P::PortRegister, AVR::B>;
    typedef AVR::Pin<PortB, 6> ocAPin;
    typedef AVR::Pin<PortB, 7> ocBPin;
    typedef AVR::Timer16Bit<3, ATMega1284P> timer_type;
    typedef timer_type::value_type value_type;
    
    using ta = AVR::ATMega1284P::Timer16Bit::TCCRA;
    static constexpr ta tccra = ta::coma0 | ta::coma1 | ta::comb0 | ta::comb1 | ta::wgm0 | ta::wgm1; 

    using tb = AVR::ATMega1284P::Timer16Bit::TCCRB;
    static constexpr tb tccrb{tb::wgm2}; 

    static constexpr value_type top = 0x3ff;
};

}
