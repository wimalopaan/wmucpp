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

#include "mcu/avr/isr.h"
#include "mcu/avr/usi.h"
#include "mcu/avr/twiaddress.h"
#include "std/array.h"
#include "std/types.h"

namespace I2C {

template<uint8_t NumberOfRegisters>
class RamRegisterMachine final {
public:
    static constexpr uint8_t size = NumberOfRegisters;
    static uint8_t& cell(uint8_t index) {
        assert(index < mData.size);
        return mData[index];        
    }
    static void process() {
    }
private:
    static std::array<uint8_t, NumberOfRegisters> mData;
};
template<uint8_t NumberOfRegisters>
std::array<uint8_t, NumberOfRegisters> RamRegisterMachine<NumberOfRegisters>::mData;

enum class State { USI_SLAVE_CHECK_ADDRESS, USI_SLAVE_SEND_DATA, USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA,
                           USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA, USI_SLAVE_REQUEST_DATA, USI_SLAVE_GET_DATA_AND_SEND_ACK};

template<typename USI, const TWI::Address& Address, typename RegisterMachine = void>
class I2CSlave final {
    I2CSlave() = delete;
    
    static constexpr auto mcu_usi = USI::mcu_usi;
    typedef typename USI::mcu_type mcu_type;
    typedef typename mcu_type::USI::USIC uc;
    typedef typename mcu_type::USI::USIS us;
    
public:
    struct I2CSlaveHandlerOvfl  : public IsrBaseHandler<AVR::ISR::Usi<0>::Overflow> {
        static inline void isr() {
            uint8_t data = 0;
            switch (state) {
            //###### Address mode: check address and send ACK (and next USI_SLAVE_SEND_DATA) if OK, else reset USI
            case State::USI_SLAVE_CHECK_ADDRESS:
                if (*mcu_usi()->usidr == 0 || (TWI::Address::fromBusValue(*mcu_usi()->usidr) == Address)) {     // If adress is either 0 or own address
                    if (*mcu_usi()->usidr & 0x01) {
                        state = State::USI_SLAVE_SEND_DATA;		// Master Write Data Mode - Slave transmit
                    }
                    else {
                        state = State::USI_SLAVE_REQUEST_DATA;		// Master Read Data Mode - Slave receive
                        index.setNaN(); // Buffer position undefined
                    }
                    USI::setSendAck();
                }
                else {
                    USI::setTwiStartConditionMode();
                }
                break;
                
                //###################################### Master Write Data Mode - Slave transmit
                // Check reply and goto USI_SLAVE_SEND_DATA if OK, 
                // else reset USI
            case State::USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
                if (*mcu_usi()->usidr) {
                    USI::setTwiStartConditionMode();	// If NACK, the master does not want more data
                    return;
                }
                [[fallthrough]];
                // From here we just drop straight into USI_SLAVE_SEND_DATA if the master sent an ACK
            case State::USI_SLAVE_SEND_DATA:
                if (!index) { 		// No buffer position given, set buffer address to 0
                    index = 0;
                }	
                else {
                    assert(index);
                    if (!(*index < RegisterMachine::size)) {
                        index = 0;
                    }
                }
                assert(index);
                *mcu_usi()->usidr = RegisterMachine::cell(*index); 	// Send data byte
                
                ++index; 					// Increment buffer address for next byte
                state = State::USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
                USI::setSendData();
                break;
                
                // Set USI to sample reply from master
                // Next USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA
            case State::USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
                state = State::USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
                USI::setReadAck();
                break;
                
                //######################################## Master Read Data Mode - Slave receive
                // Set USI to sample data from master,
                // Next USI_SLAVE_GET_DATA_AND_SEND_ACK
            case State::USI_SLAVE_REQUEST_DATA:
                state = State::USI_SLAVE_GET_DATA_AND_SEND_ACK;
                USI::setReadData();
                break;
                
                // Copy data from USIDR and send ACK
                // Next USI_SLAVE_REQUEST_DATA
            case State::USI_SLAVE_GET_DATA_AND_SEND_ACK:
                data = *mcu_usi()->usidr; 					// Read data received
                if (!index) { 		// First access, read buffer position
                    if (data < RegisterMachine::size) {		// Check if address within buffer size
                        index = data; 		// Set position as received
                    }
                    else {
                        index = 0; 			// Set address to 0
                    }				
                }
                else {					// Ongoing access, receive data
                    assert(index);
                    if (!(*index < RegisterMachine::size)) {
                        index = 0;
                    }
                    assert(index);
                    RegisterMachine::cell(*index) = data; 				// Write data to buffer
                    ++index; 							// Increment buffer address for next write access
                }
                state = State::USI_SLAVE_REQUEST_DATA;	// Next USI_SLAVE_REQUEST_DATA
                USI::setSendAck();
                break;
            default:
                assert(false);
                break;
            }
        }
    };
    struct I2CSlaveHandlerStart  : public IsrBaseHandler<AVR::ISR::Usi<0>::Start> {
        static inline void isr() {
            state = State::USI_SLAVE_CHECK_ADDRESS;        
            USI::mosi_pin::template dir<AVR::Input>();
            
            // Wait for SCL to go low to ensure the Start Condition has completed (the
            // Start detector will hold SCL low ) - if a Stop Condition arises then leave
            // The interrupt to prevent waiting forever - don't use USISR to test for Stop
            // Condition as in Application Note AVR312 because the Stop Condition Flag is
            // going to be set from the last TWI sequence
            
            while (USI::clock_pin::read() && !USI::mosi_pin::read());// SCL his high and SDA is low
            
            if (!USI::clock_pin::read()) {	// A Stop Condition did not occur
                mcu_usi()->usicr.template set<uc::sie | uc::oie | uc::wm1 | uc::wm0 | uc::cs1>();
            }
            else {	// A Stop Condition did occur
                mcu_usi()->usicr.template set<uc::sie | uc::wm1 | uc::cs1>();
            } 
            mcu_usi()->usisr.template set<us::sif | us::oif | us::pf | us::dc>();
        }
    };
    static inline void init() {
        USI::template init<AVR::I2C<mcu_type>>();
        state = State::USI_SLAVE_CHECK_ADDRESS;
    }
private:
    static volatile uint_NaN<uint8_t> index;
    static volatile State state;
};
template<typename USI, const TWI::Address& Address, typename RegisterMachine>
volatile State I2CSlave<USI, Address, RegisterMachine>::state = State::USI_SLAVE_CHECK_ADDRESS;

template<typename USI, const TWI::Address& Address, typename RegisterMachine>
volatile uint_NaN<uint8_t> I2CSlave<USI, Address, RegisterMachine>::index;

}