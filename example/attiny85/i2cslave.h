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

#include <stdint.h>


/*

	Description:
	
	The slave device acts like a I2C-EEPROM.
	
	Write data from master to slave (into slave rxbuffer):

        1. Master sends slave address (bit 7-1) + r/w flag (bit 0), which must be set to 0
        2. Master sends address of where to start with writing data (one byte)
		3. Master sends data to rxbuffer[buffer address]

    Read data from the slave (from slave txbuffer):

	    1. Master sends slave address (bit 7-1) + r/w flag (bit 0), which must be set to 0
		2. Master sends address of where to start reading data (one byte)
		3. Master sends slave address (bit 7-1) + r/w flag (bit 0), which must be set to 1
		4. Master waits for callback, demanding the slave to send data starting with txbuffer[buffer address]

	Info:
		- You have to change the buffer_size in the usiTwiSlave.h file
		- Buffer address is automatically incremented
		- If buffer address > buffersize -> start with buffer address 0x00
	
//############################################################################*/

#ifndef _USI_TWI_SLAVE_H_
#define _USI_TWI_SLAVE_H_

//##################################################################### includes

#include <stdbool.h>

//################################################################### prototypes

void    usiTwiSlaveInit(uint8_t ownAddress);	// send slave address

//#################################################################### variables

#define buffer_size 16						     //in bytes (2..254), change ONLY here!!!!!


volatile uint8_t rxbuffer[buffer_size];         // Buffer to write data received from the master
volatile uint8_t txbuffer[buffer_size];			// Transmission buffer to be read from the master
volatile uint8_t buffer_adr; 					// Virtual buffer address register


#if 	(buffer_size > 254)
		#error Buffer to big! Maximal 254 Bytes.
		
#elif 	(buffer_size < 2)
		#error Buffer to small! mindestens 2 Bytes!
#endif

//##############################################################################

#endif  // ifndef _USI_TWI_SLAVE_H_

