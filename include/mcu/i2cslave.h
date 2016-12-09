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

#include "mcu/avr/isr.h"
#include "mcu/avr/usi.h"
#include "mcu/avr/twiaddress.h"
#include "std/array.h"

template<typename USI, const TWI::Address& Address, uint8_t Size>
class I2CSlave final {
    I2CSlave() = delete;
public:
    struct I2CSlaveHandlerOvfl  : public IsrBaseHandler<AVR::ISR::Usi<0>::Overflow> {
        static inline void isr() {
            
        }
        
    };
    struct I2CSlaveHandlerStart  : public IsrBaseHandler<AVR::ISR::Usi<0>::Start> {
        static inline void isr() {
            
        }
        
    };
    static inline void init() {
        USI::init();
    }
private:
    static std::array<uint8_t, Size> data;
};
