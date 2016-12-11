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

// fixme: eränzen
struct I2C {
    template<typename SDAPin, typename SCLPin>
    static void init() {
        
    }
};
struct SPI {
    template<typename SDAPin, typename SCLPin>
    static void init() {
        
    }
    
};

template<uint8_t N, typename MCU = DefaultMcuType> struct UsiPort;

template<>
struct UsiPort<0, AVR::ATTiny84> {
    using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
    
    typedef AVR::Pin<PortA, 4> clock_pin;
    typedef AVR::Pin<PortA, 5> miso_pin;
    typedef AVR::Pin<PortA, 6> mosi_pin;
};

template<uint8_t N, typename SSPin = void, typename Inserter = void, typename MCU = DefaultMcuType>
class Usi final : public IsrBaseHandler<AVR::ISR::Usi<0>::Overflow> {
    static constexpr auto mcu_usi = getBaseAddr<typename MCU::USI, N>;
    
    Usi() = delete;
    
public:
    template<typename Mode = SPI>
    static void init() {
        if constexpr(!std::is_same<SSPin, void>::value) {
            SSPin::template dir<Input>();
            SSPin::pullup();
        }

        Mode::template init<typename UsiPort<N, MCU>::mosi_pin, typename UsiPort<N, MCU>::clock_pin>();
        
        UsiPort<N, MCU>::clock_pin::template dir<Input>();
        UsiPort<N, MCU>::clock_pin::pullup();
        UsiPort<N, MCU>::mosi_pin::template dir<Input>();
        UsiPort<N, MCU>::mosi_pin::pullup();

        if constexpr(std::is_same<typename UsiPort<N, MCU>::miso_pin, SSPin>::value) {
            UsiPort<N, MCU>::miso_pin::template dir<Input>();
            UsiPort<N, MCU>::miso_pin::pullup();
        }
        else {
            UsiPort<N, MCU>::miso_pin::template dir<Output>();
        }
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
        mcu_usi()->usisr = _BV(USIOIF);        
        if constexpr(!std::is_same<Inserter, void>::value) {
            Inserter::insert(mcu_usi()->usibr);
        }
    }
private:
};

}
