/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

#include "compat/twi.h"

namespace TWI {

template<typename TWIMaster>
class MasterAsync {
    // todo: make async
};

template<uint8_t N, typename MCU = DefaultMcuType>
class Master final {
public:
    static constexpr const auto mcuTwi = AVR::getBaseAddr<typename MCU::TWI, N>;
    
    // todo: use mcuTwi
    static void init() {
        // todo: calculate rate
        TWSR |= _BV(TWPS1) | _BV(TWPS0);                         
        TWBR = 128;  
    }

    template<uint8_t Address, bool Write>
    static constexpr uint8_t rwAddress() {
        if constexpr(Write) {
            return Address << 1;
        }
        else {
            return (Address << 1) | 0x01;
        }
    }
    
    template<uint8_t Address, bool Write>
    static bool start() {
        uint8_t   twst;
    
        // send START condition
        TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
        // wait until transmission completed
        while(!(TWCR & (1<<TWINT)));
    
        // check value of TWI Status Register. Mask prescaler bits.
        twst = TW_STATUS & 0xF8;
        if ( (twst != TW_START) && (twst != TW_REP_START)) {
            return false;
        }
    
        // send device address
        TWDR = rwAddress<Address, Write>();

        TWCR = (1<<TWINT) | (1<<TWEN);
    
        // wail until transmission completed and ACK/NACK has been received
        while(!(TWCR & (1<<TWINT)));
    
        // check value of TWI Status Register. Mask prescaler bits.
        twst = TW_STATUS & 0xF8;
        if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) {
            return false;
        }
        return true;
    }
    
    static void stop() {
        /* send stop condition */
         TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
         
         // wait until stop condition is executed and bus released
         while(TWCR & (1<<TWSTO));
    }
    
    static bool write(uint8_t data) {
        uint8_t   twst = 0;
        
        // send data to the previously addressed device
        TWDR = data;
        TWCR = (1<<TWINT) | (1<<TWEN);
    
        // wait until transmission completed
        while(!(TWCR & (1<<TWINT)));
    
        // check value of TWI Status Register. Mask prescaler bits
        twst = TW_STATUS & 0xF8;
        if( twst != TW_MT_DATA_ACK) return false;
        return true;       
    }
    
    static uint8_t read() {
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
        while(!(TWCR & (1<<TWINT)));    
    
        return TWDR;    
    }
    
    static uint8_t readBeforeStop() {
        TWCR = (1<<TWINT) | (1<<TWEN);
        while(!(TWCR & (1<<TWINT)));
        
        return TWDR;
    }
    
private:
};


}
