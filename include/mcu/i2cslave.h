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

enum class State { USI_SLAVE_CHECK_ADDRESS, USI_SLAVE_SEND_DATA, USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA,
                           USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA, USI_SLAVE_REQUEST_DATA, USI_SLAVE_GET_DATA_AND_SEND_ACK};

template<typename USI, const TWI::Address& Address, uint8_t Size = 8, typename Client = void>
class I2CSlave final {
    I2CSlave() = delete;
    
    static constexpr auto mcu_usi = USI::mcu_usi;
    typedef typename USI::mcu_type mcu_type;
    typedef typename mcu_type::USI::USIC uc;
    typedef typename mcu_type::USI::USIS us;
    
public:
    struct I2CSlaveHandlerOvfl  : public IsrBaseHandler<AVR::ISR::Usi<0>::Overflow> {
        static inline void isr() {
            std::byte data{0};
            switch (mState) {
            //###### Address mode: check address and send ACK (and next USI_SLAVE_SEND_DATA) if OK, else reset USI
            case State::USI_SLAVE_CHECK_ADDRESS:
                if ((*mcu_usi()->usidr == std::byte{0}) || (TWI::Address::fromBusValue(*mcu_usi()->usidr) == Address)) {     // If adress is either 0 or own address
                    if (std::any(*mcu_usi()->usidr & std::byte{0x01})) {
                        mState = State::USI_SLAVE_SEND_DATA;		// Master Write Data Mode - Slave transmit
                    }
                    else {
                        mState = State::USI_SLAVE_REQUEST_DATA;		// Master Read Data Mode - Slave receive
                        mIndex.setNaN(); // Buffer position undefined
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
                if (std::any(*mcu_usi()->usidr)) {
                    USI::setTwiStartConditionMode();	// If NACK, the master does not want more data
                    return;
                }
                [[fallthrough]];
                // From here we just drop straight into USI_SLAVE_SEND_DATA if the master sent an ACK
            case State::USI_SLAVE_SEND_DATA:
                if (!mIndex) { 		// No buffer position given, set buffer address to 0
                    mIndex = 0;
                }	
                else {
                    assert(mIndex);
                    if (!(*mIndex < Size)) {
                        mIndex = 0;
                    }
                }
                assert(mIndex);
                *mcu_usi()->usidr = mRegisters[*mIndex]; 	// Send data byte
                
                ++mIndex; 					// Increment buffer address for next byte
                mState = State::USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
                USI::setSendData();
                break;
                
                // Set USI to sample reply from master
                // Next USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA
            case State::USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
                mState = State::USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
                USI::setReadAck();
                break;
                
                //######################################## Master Read Data Mode - Slave receive
                // Set USI to sample data from master,
                // Next USI_SLAVE_GET_DATA_AND_SEND_ACK
            case State::USI_SLAVE_REQUEST_DATA:
                mState = State::USI_SLAVE_GET_DATA_AND_SEND_ACK;
                USI::setReadData();
                break;
                
                // Copy data from USIDR and send ACK
                // Next USI_SLAVE_REQUEST_DATA
            case State::USI_SLAVE_GET_DATA_AND_SEND_ACK:
                data = *mcu_usi()->usidr; 					// Read data received
                if (!mIndex) { 		// First access, read buffer position
                    if (std::to_integer<uint8_t>(data) < Size) {		// Check if address within buffer size
                        mIndex = std::to_integer<uint8_t>(data); 		// Set position as received
                    }
                    else {
                        mIndex = 0; 			// Set address to 0
                    }				
                }
                else {					// Ongoing access, receive data
                    assert(mIndex);
                    if (!(*mIndex < Size)) {
                        mIndex = 0;
                    }
                    assert(mIndex);
                    mRegisters[*mIndex] = data; 				// Write data to buffer
                    ++mIndex; 							// Increment buffer address for next write access
                    if constexpr(!std::is_same<Client, void>::value) {
                        Client::notify();
                    }
                }
                mState = State::USI_SLAVE_REQUEST_DATA;	// Next USI_SLAVE_REQUEST_DATA
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
            mState = State::USI_SLAVE_CHECK_ADDRESS;        
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
        mState = State::USI_SLAVE_CHECK_ADDRESS;
    }
    static auto& registers() {
        return mRegisters;
    }
private:
    inline static volatile std::array<std::byte, Size> mRegisters;
    inline static volatile uint_NaN<uint8_t> mIndex;
    inline static volatile State mState = State::USI_SLAVE_CHECK_ADDRESS;
};

}