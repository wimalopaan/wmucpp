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

#include <string.h> 
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "ow.h"

uint8_t onewire_reset(void) {
    uint8_t rc=ONEWIRE_OK;

    ONEWIRE_LOW
    _delay_us(480);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ONEWIRE_TRISTATE
        _delay_us(66);
        if(ONEWIRE_READ) {         // no presence pulse detect
            rc = ONEWIRE_NO_PRESENCE;
        }
    }

    _delay_us(480);
    if(!ONEWIRE_READ) {        // bus short circuit to GND
        rc = ONEWIRE_GND_SHORT;
    }
    return rc;
}

void onewire_write_bit(uint8_t wrbit) {

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if ((wrbit & 1))   {        // write 1
            ONEWIRE_LOW
            _delay_us(3);
            ONEWIRE_TRISTATE
            _delay_us(97);
        } else {                    // write 0
            ONEWIRE_LOW
            _delay_us(80);
            ONEWIRE_TRISTATE          
            _delay_us(20);
        }
    }
}

uint8_t onewire_read_bit(void) {
    uint8_t readbit;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {    
        ONEWIRE_LOW
        _delay_us(3);
        ONEWIRE_TRISTATE
        _delay_us(12);
        readbit = ONEWIRE_READ;
        _delay_us(85);
    }

    if (readbit) {
        return 1;
    } else {
        return 0;
    }   
}

uint8_t onewire_read_byte(void) {
    uint8_t data=0;
    uint8_t i;

    for (i=0; i<8; i++) {
        data >>= 1;         // LSB first on OneWire
        if (onewire_read_bit()) {
            data |= 0x80;
        }
    }
    return data;
}

void onewire_write_byte(uint8_t data) {
    uint8_t i;

    for (i=0; i<8; i++) {       
        // LSB first on OneWire
        // no need for masking, LSB is masked inside function
        onewire_write_bit(data);
        data >>= 1;
    }
}

void onewire_search_init(uint8_t buffer[8]) {
    memset(buffer, 0, 8);
    onewire_search(NULL, 0);
}

uint8_t onewire_search_rom(uint8_t buffer[8]) {
    return onewire_search(buffer, ONEWIRE_SEARCH_ROM);
}

uint8_t onewire_alarm_search(uint8_t buffer[8]) {
    return onewire_search(buffer, ONEWIRE_ALARM_SEARCH);
}

uint8_t onewire_search(uint8_t buffer[8], uint8_t cmd) {
    uint8_t mask, i, j, bit, rom_tmp, rc;
    uint8_t max_conf_zero=0;        // last bit conflict that was resolved to zero
    static uint8_t max_conf_old;    // last bit conflict that was resolved to zero in last scan
    uint8_t branch_flag=0;          // indicate new scan branch, new ROM code found  

    if (buffer == NULL) {    // init search
        max_conf_old=64;
        return ONEWIRE_OK;
    }

    rc = onewire_reset();
    if (rc) {
        return rc;
    } else {
        onewire_write_byte(cmd);
        rom_tmp  = buffer[0];
        i=0;
        mask=1;
        // scan all 64 ROM bits
        for(j=0; j<64; j++) {
            bit  = onewire_read_bit();      // bit
            bit |= onewire_read_bit()<<1;   // inverted bit

            switch(bit) {
                case 0:     // bit conflict, more than one device with different bit
                    if (j < max_conf_old) {         // below last zero branch conflict level, keep current bit
                        if (rom_tmp & mask) {       // last bit was 1
                            bit = 1;
                        } else {                    // last bit was 0
                            bit = 0;
                            max_conf_zero = j;
                            branch_flag = 1;
                        }                        
                    } else if (j == max_conf_old) { // last zero branch conflict, now enter new path
                        bit = 1;
                    } else {                        // above last scan conflict level
                        bit = 0;                    // scan 0 branch first
                        max_conf_zero = j;
                        branch_flag = 1;
                    }
                break;

                case 1:
                // no break

                case 2: // no bit conflict
                    // do nothing, just go on with current bit
                break;

                case 3:     // no response
                    return ONEWIRE_SCAN_ERROR;
                break;
            }

            // write bit to OneWire and ROM code

            if (bit & 1) {
                onewire_write_bit(1);
                rom_tmp |= mask;
            } else {
                onewire_write_bit(0);
                rom_tmp &= ~mask;
            }
            
            mask <<= 1;
            if (mask == 0) {
                mask = 1;
                buffer[i] = rom_tmp;            // update tmp data
                i++;
                if (i<8) {
                    rom_tmp  = buffer[i];       // read new data
                }
            }
        }
    }

    max_conf_old = max_conf_zero;

    if (onewire_crc(buffer, 8)) {
        return ONEWIRE_CRC_ERROR;
    } else if (branch_flag) {
        return ONEWIRE_OK;
    } else {
        return ONEWIRE_LAST_CODE;
    }
}

uint8_t onewire_match_rom(const uint8_t rom[8]) {
    uint8_t i, rc;

    rc = onewire_reset();
    if (rc) {
        return rc;
    } else {
        onewire_write_byte(ONEWIRE_MATCH_ROM);
        for (i=0; i<8; i++) {
            onewire_write_byte(rom[i]);
        }
    }
    return ONEWIRE_OK;
}

uint8_t onewire_skip_rom(void) {
    uint8_t rc;

    rc = onewire_reset();
    if (rc) {
        return rc;
    } else {
        onewire_write_byte(ONEWIRE_SKIP_ROM);
    }
    return ONEWIRE_OK;
}

uint8_t onewire_read_rom(uint8_t rom[8]) {
    uint8_t i, rc;
    
    rc = onewire_reset(); 
    if (rc) {
        return rc;
    } else {
        onewire_write_byte(ONEWIRE_READ_ROM);
        for (i=0; i<8; i++) {
            rom[i] = onewire_read_byte();
        }

        if(onewire_crc(rom, 8)) {
            return ONEWIRE_CRC_ERROR;
        }
    }
    return ONEWIRE_OK;
}

/*-----------------------------------------------------------------------------

    calculate CRC over data array
    nibble wide processing, ~3x faster than serial version
    CRC calculation of 8 Bytes takes ~0.3ms@1MHz

    CRC polynon is x^8 + x^5 + x^4 + 1 = 0x31

    Attention! This CRC is reversed, CRC is shifted right with new data
    comming in at the MSB! That's why the polynom is mirrored!
    So the real polynom is 0x8C!!!

    return: CRC

    usage:

    check CRC of received data:

    When the array includes the received CRC (last byte), the calculated CRC
    will be zero if no transmission error took place.

    When the array does not include the received CRC, the byte after the
    last valid data must be zero. The calculated CRC will be returned and must
    be compared to the received CRC.

    generate CRC for transmit data:

    When a CRC for transmission is to be calculated, add a zero after the last
    valid data and copy the CRC value there after this functions returns.

-----------------------------------------------------------------------------*/

uint8_t onewire_crc(const uint8_t *data, uint8_t cnt) {

    static const uint8_t crc_table[16] PROGMEM = 
    {0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
     0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74};
    
    uint8_t crc, i, tmp, zerocheck;

    // nibble based CRC calculation,
    // good trade off between speed and memory usage

    // first byte is not changed, since CRC is initialized with 0
    crc = *data++;
    zerocheck = crc;        
    cnt--;

    for(; cnt>0; cnt--) {
        tmp = *data++;                        // next byte
        zerocheck |= tmp;

        i = crc & 0x0F;
        crc = (crc >> 4) | (tmp << 4);        // shift in next nibble
        crc ^= pgm_read_byte(&crc_table[i]);  // apply polynom

        i = crc & 0x0F;
        crc = (crc >> 4) | (tmp & 0xF0);      // shift in next nibble
        crc ^= pgm_read_byte(&crc_table[i]);  // apply polynom
    }

    if (!zerocheck) {        // all data was zero, this is an error!
        return 0xFF;
    } else {
        return crc;
    }
}

/*-----------------------------------------------------------------------------

    calculate CRC over data array
    serial processing, slower but slightly less programm memory usage
    CRC calculation of 8 Bytes takes ~1ms@1MHz

    CRC polynon is x^8 + x^5 + x^4 + 1 = 0x31

    Attention! This CRC is reversed, CRC is shifted right with new data
    comming in at the MSB! That's why the polynom is mirrored!
    So the real polynom is 0x8C!!!

    return: CRC

    usage:

    check CRC of received data:

    When the array includes the received CRC (last byte), the calculated CRC
    will be zero if no transmission error took place.

    When the array does not include the received CRC, the byte after the
    last valid data must be zero. The calculated CRC will be returned and must
    be compared to the received CRC.

    generate CRC for transmit data:

    When a CRC for transmission is to be calculated, add a zero after the last
    valid data and copy the CRC value there after this functions returns.

-----------------------------------------------------------------------------*/

uint8_t onewire_crc_serial(const uint8_t *data, uint8_t cnt) {
    
    uint8_t crc, i, tmp, zerocheck;
    const uint8_t poly=0x8C;

    // first byte is not changed, since CRC is initialized with 0
    crc = *data++;   
    zerocheck = crc;     
    cnt--;

    for(; cnt>0; cnt--) {
        tmp = *data++;                        // next byte
        zerocheck |= tmp;
        for (i=0; i<8; i++) {
          if (crc & 1) {
            crc >>= 1;            
            if (tmp & 1) crc |= 0x80;        // copy LSB to MSB
            crc ^= poly;
          } else {
            crc >>= 1;            
            if (tmp & 1) crc |= 0x80;        // copy LSB to MSB
          }
          tmp >>=1;
        }
    }

    if (!zerocheck) {        // all data was zero, this is an error!
        return 0xFF;
    } else {
        return crc;
    }
}

