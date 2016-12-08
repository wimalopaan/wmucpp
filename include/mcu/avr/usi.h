/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/isr.h"

#include "std/traits.h"

namespace AVR {

template<uint8_t N, typename MCU = DefaultMcuType> struct UsiPort;

template<>
struct UsiPort<0, AVR::ATTiny84> {
    using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
    
    typedef AVR::Pin<PortA, 4> clock_pin;
    typedef AVR::Pin<PortA, 5> miso_pin;
    typedef AVR::Pin<PortA, 6> mosi_pin;
};

template<uint8_t N, typename SSPin = void, typename MCU = DefaultMcuType>
class Usi final : public IsrBaseHandler<AVR::ISR::Usi<0>::Overflow> {
    static constexpr auto mcu_usi = getBaseAddr<typename MCU::USI, N>;
    
    Usi() = delete;
    
public:
    static void init() {
        if constexpr(!std::is_same<SSPin, void>::value) {
            SSPin::template dir<Input>();
            SSPin::pullup();
        }

        UsiPort<N, MCU>::clock_pin::template dir<Input>();
        UsiPort<N, MCU>::clock_pin::pullup();
        UsiPort<N, MCU>::mosi_pin::template dir<Input>();
        UsiPort<N, MCU>::mosi_pin::pullup();
        UsiPort<N, MCU>::miso_pin::template dir<Output>();
        
        mcu_usi()->usicr = _BV(USIWM0) | _BV(USICS1) | _BV(USIOIE);        
        mcu_usi()->usisr = _BV(USIOIF);        
    }

    static bool select() {
        if constexpr(std::is_same<SSPin, void>::value) {
            return true;
        }
        return !SSPin::read();
    }
    
    static void put(uint8_t value) {
        mcu_usi()->usidr = value;
    }
    
    static uint8_t get() {
        while(!(mcu_usi()->usisr & _BV(USIOIF)));
        return mcu_usi()->usibr;
    }
    
    static void isr() {
    }

private:
};

}
