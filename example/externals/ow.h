/*****************************************************************************
 
 OneWire (tm) library
 
 Copyright (C) 2016 Falk Brunner

*****************************************************************************/
 
/*
* ----------------------------------------------------------------------------
* "THE BEER-WARE LICENSE" (Revision 42):
* <Falk.Brunner@gmx.de> wrote this file. As long as you retain this notice you
* can do whatever you want with this stuff. If we meet some day, and you think
* this stuff is worth it, you can buy me a beer in return. Falk Brunner
* ----------------------------------------------------------------------------
*/
 
/**
\mainpage

 \par onewire.h - onewire library

 \author Falk Brunner

 \version 1.00
 
 \par License:
 \subpage LICENSE "Beerware License"
 
 \par Files:
    \subpage onewire.h \n
    \subpage onewire.c \n

 \par Developed on AVR plattform, but can be used with any other
      AVR-Studio 4.18, WinAVR20100110 (avr-gcc 4.3.3) \n
      Atomic access must be adapted to other plattforms!
      Onewire operation is interrupt proof. During one single bit transmission
      and onewire reset, interrupts are disables for 100us. Between single bits
      interrupts are enabled to allow serviceing of interrupts.
 
 \par Example:
 \include ./onewire_demo.ino
 \page LICENSE Beerware License
 \include ./beerware.txt
 \page ONEWIRE.H onewire.h
 \include ./onewire.h
 \page ONEWIRE.C onewire.c
 \include ./onewire.c
*/

#ifndef ONEWIRE_H_
#define ONEWIRE_H_

#include <avr/io.h>

/** \defgroup ONEWIRE_CONFIGURATION ONEWIRE CONFIGURATION
  static configuration of IO port and pin
*/
/*@{*/
#define ONEWIRE_BIT  PA5
#define ONEWIRE_PIN  PINA
#define ONEWIRE_PORT PORTA
#define ONEWIRE_DDR  DDRA
/*@}*/

/** \defgroup ONEWIRE_CONTROL ONEWIRE CONTROL
  control macros for strong pull up, used for parasitic power supply
*/
/*@{*/
#define ONEWIRE_STRONG_PU_ON  ONEWIRE_PORT |= ONEWIRE_MASK; ONEWIRE_DDR |= ONEWIRE_MASK;
#define ONEWIRE_STRONG_PU_OFF ONEWIRE_DDR  &= ~ONEWIRE_MASK;
/*@}*/

/** \defgroup ONEWIRE_INTERNAL_DEFINE ONEWIRE INTERNAL DEFINES
 internal used defines, used for easy adaption to other CPUs
*/
/*@{*/
#define ONEWIRE_MASK      (1<<ONEWIRE_BIT)            //
#define ONEWIRE_LOW       ONEWIRE_PORT &= ~ONEWIRE_MASK; ONEWIRE_DDR |= ONEWIRE_MASK;
#define ONEWIRE_TRISTATE  ONEWIRE_DDR &= ~ONEWIRE_MASK;
#define ONEWIRE_READ      (ONEWIRE_PIN & ONEWIRE_MASK)

/*@}*/

/** \defgroup ONEWIRE_COMMANDS ONEWIRE COMMANDS
  command codes for onewire
*/
/*@{*/
#define ONEWIRE_MATCH_ROM       0x55
#define ONEWIRE_SEARCH_ROM      0xF0
#define ONEWIRE_SKIP_ROM        0xCC
#define ONEWIRE_READ_ROM        0x33
#define ONEWIRE_ALARM_SEARCH    0xEC
/*@}*/

/** \defgroup ONEWIRE_RETURN ONEWIRE RETURN CODES
  return codes
*/
/*@{*/
#define ONEWIRE_OK           0        // no error
#define ONEWIRE_NO_PRESENCE  1        // no presence pulse detected during onewire reset
#define ONEWIRE_CRC_ERROR    2        // crc error in data reception
#define ONEWIRE_SCAN_ERROR   3        // scan error during ROM scan
#define ONEWIRE_LAST_CODE    4        // last rom code scaned (no error, just info)
#define ONEWIRE_GND_SHORT    5        // bus short circuit to GND
/*@}*/

/** \defgroup ONEWIRE_FUNCTIONS ONEWIRE FUNCTIONS 
  * standard functions
  */
/*@{*/ 

/**
 \brief OneWire reset
 \brief reset onewire bus 
 \param none
 \return error code
 \return ONEWIRE_OK no error
 \return ONEWIRE_NO_PRESENCE no presence pulse detected
 \return ONEWIRE_GND_SHORT bus short circuit to ground
 */   

uint8_t onewire_reset(void);

/**
 \brief read one byte
 \param none
 \return read data byte
 */

uint8_t onewire_read_byte(void);

/**
 \brief write one byte
 \param data byte to write
 \return none
 */

void onewire_write_byte(uint8_t data);

/**
 \brief init rom search buffer and internal variables
 \param buffer[8] pointer to buffer array
 \return none
 */

void onewire_search_init(uint8_t buffer[8]);

/**
 \brief scan OneWire bus for devices with active alarm flag
 \brief call onewire_search_init() before first call of this function
 \brief after each call, the buffer contains the new scaned ROM code
 \param buffer[8] pointer to buffer array
 \return error code
 \return ONEWIRE_OK new ROM code scanned successfully
 \return ONEWIRE_NO_PRESENCE no bus response during reset
 \return ONEWIRE_GND_SHORT bus short circuit to GND
 \return ONEWIRE_CRC_ERROR CRC error in received rom code
 \return ONEWIRE_SCAN_ERROR no bus response during scan
 \return ONEWIRE_LAST_CODE last scan, no more codes available
 */

uint8_t onewire_alarm_search(uint8_t buffer[8]);

/**
 \brief scan OneWire bus for ROMs
 \brief call onewire_search_init() before first call of this function
 \brief after each call, the buffer contains the new scaned ROM code
 \param buffer[8] pointer to buffer array
 \return error code
 \return ONEWIRE_OK new ROM code scanned successfully
 \return ONEWIRE_NO_PRESENCE no bus response during reset
 \return ONEWIRE_GND_SHORT bus short circuit to GND
 \return ONEWIRE_CRC_ERROR CRC error in received rom code
 \return ONEWIRE_SCAN_ERROR no bus response during scan
 \return ONEWIRE_LAST_CODE last scan, no more codes available
 */

uint8_t onewire_search_rom(uint8_t buffer[8]);

/**
 \brief select device on bus
 \param rom[8] pointer to ROM ID
 \return error code
 \return ONEWIRE_OK code access successfully
 \return ONEWIRE_NO_PRESENCE no bus response during reset
 \return ONEWIRE_GND_SHORT bus short circuit to GND
 */

uint8_t onewire_match_rom(const uint8_t rom[8]);

/**
 \brief read ROM ID of device
 \brief can only be used for a single device on bus
 \param rom[8] pointer to buffer array
 \return error code
 \return ONEWIRE_OK code access successfully
 \return ONEWIRE_NO_PRESENCE no bus response during reset
 \return ONEWIRE_GND_SHORT bus short circuit to GND
 \return ONEWIRE_CRC_ERROR CRC error in rom data
 */

uint8_t onewire_read_rom(uint8_t rom[8]);

/**
 \brief select device on bus
 \brief can only be used for a single device on bus
 \param none
 \return error code
 \return ONEWIRE_OK code access successfully
 \return ONEWIRE_NO_PRESENCE no bus response during reset
 \return ONEWIRE_GND_SHORT bus short circuit to GND
 */

uint8_t onewire_skip_rom(void);

/**
 \brief calculate CRC over data array, fast version, 0.3ms for 8 bytes @1MHz
 \param *data pointer to buffer array
 \param cnt number of data bytes
 \return calculated CRC
 \return Calculating a CRC over a data array including the received CRC
 \return (last byte) will return zero in case of a valid CRC
 \return Calculating a CRC over a data array without the received CRC
 \return (last byte zero, placeholder for CRC) will return the calculated CRC.
 \return If all data bytes are zero, 0xFF is returned to indicate an error
 */

uint8_t onewire_crc(const uint8_t *data, uint8_t cnt);

/**
 \brief calculate CRC over data array, serial version, 1ms for 8 bytes @1MHz
 \param *data pointer to buffer array
 \param cnt number of data bytes
 \return calculated CRC
 \return Calculating a CRC over a data array including the received CRC
 \return (last byte) will return zero in case of a valid CRC
 \return Calculating a CRC over a data array without the received CRC
 \return (last byte zero, placeholder for CRC) will return the calculated CRC.
 \return If all data bytes are zero, 0xFF is returned to indicate an error
 */

uint8_t onewire_crc_serial(const uint8_t *data, uint8_t cnt);

/*@}*/

/** \defgroup ONEWIRE_PRIVATE ONEWIRE PRIVATE FUNCTIONS
*/
/*@{*/

/**
 \brief write one bit to bus 
 \param data write data (bit#0)
 \return none
 */

void onewire_write_bit(uint8_t data);

/**
 \brief read one bit from bus 
 \param none
 \return read data (bit #0)
 */

uint8_t onewire_read_bit(void);

/**
 \brief scan OneWire bus for normal ROM or alarm search
 \brief call onewire_search_init() before first call of this function
 \brief after each call, the buffer contains the new scaned ROM code
 \param buffer[8] pointer to buffer array
 \param cmd onewire search command
 \return error code
 \return ONEWIRE_OK new ROM code scanned successfully
 \return ONEWIRE_NO_PRESENCE no bus response during reset
 \return ONEWIRE_GND_SHORT bus short circuit to GND
 \return ONEWIRE_CRC_ERROR CRC error in received rom code
 \return ONEWIRE_SCAN_ERROR no bus response during scan
 \return ONEWIRE_LAST_CODE last scan, no more codes available
 */

uint8_t onewire_search(uint8_t buffer[8], uint8_t cmd);

#endif
