/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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


// 20MHz full-swing
// sudo avrdude -p atmega1284P -P usb -c avrisp2 -U lfuse:w:0xf7:m -U hfuse:w:0xd1:m -U efuse:w:0xfc:m

//#define MEM
#define OW
//#define DCF
//#define I2C
//#define HOTT
#define BTerm
//#define I2CInt

#include <stdint.h>
#include <string.h>

#include "mcu/ports.h"
#include "mcu/avr/isr.h"

#include "hal/softspimaster.h"
#include "hal/bufferedstream.h"
#include "hal/alarmtimer.h"
#include "hal/constantrate.h"

#include "external/ws2812.h"

#ifdef DCF
# include "external/dcf77.h"
#endif

#ifdef OW
# include "external/onewire.h"
# include "external/ds18b20.h"
#endif

#ifdef I2C
# include "external/ds1307.h"
# include "external/i2cram.h"
#endif

#ifdef HOTT
# include "external/hott/hott.h"
#endif

#include "console.h"

#ifdef MEM
# include "util/memory.h"
#endif

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using SoftSPIData = AVR::Pin<PortB, 1>;
using SoftSPIClock = AVR::Pin<PortB, 0>;
using SoftSPISS = AVR::Pin<PortC, 3>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;
using terminal = SSpi0;
//using bufferedTerminal = BufferedStream<SSpi0, 512>;

using oneWirePin = AVR::Pin<PortC, 7>;
using oneWireMaster = OneWire::Master<oneWirePin, OneWire::Normal>;
//using oneWireMaster = OneWire::Master<oneWirePin, OneWire::Normal, true, true>;
//using oneWireMasterAsync = OneWire::MasterAsync<oneWireMaster, constantRatePeriod>;
//using ds18b20 = DS18B20<oneWireMasterAsync>;
std::array<OneWire::ow_rom_t, 5> dsIds;

#include "ow.h"

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

#define MAX_ROMS 10
typedef uint8_t rom_t[MAX_ROMS][8];
uint8_t buffer[9] = {0x10,  0x1D, 0x9E, 0x09, 0x02, 0x08, 0x00, 0x55, 0x00};  // init values just for test & debugging
rom_t roms;

int scan_bus(rom_t rom_list, uint8_t cmd) {
  uint8_t i = 0, rc = ONEWIRE_NO_PRESENCE;

  onewire_search_init(buffer);

  rc=ONEWIRE_OK;
  for (i = 0; (i < MAX_ROMS) && (rc == ONEWIRE_OK); i++) {
    rc = onewire_search(buffer, cmd);
   
    if (rc == ONEWIRE_OK || rc == ONEWIRE_LAST_CODE ) {
        OneWire::Rom rom;
        for(int i = 0; i < 8; ++i) {
            rom[i] = buffer[i];
        }
        std::cout << rom << std::endl;
//      print_rom(buffer, i);
      memcpy(rom_list[i], buffer, 8);
    } else {
      // error
      switch (rc) {
        case ONEWIRE_NO_PRESENCE: 
          std::cout << "No response on bus!" << std::endl;; 
          break;
        case ONEWIRE_GND_SHORT:   
          std::cout << "Bus shorted to GND!" << std::endl;; 
          break;

        case ONEWIRE_CRC_ERROR:
          std::cout << "CRC error!" << std::endl;
//          print_rom(buffer, i);
        break;

        case ONEWIRE_SCAN_ERROR:
          if (cmd == ONEWIRE_SEARCH_ROM) { 
            std::cout << "Scan error!" << std::endl;
          } else {
            return 0;  // no alarm device found
          }
        break;             
      }
    }
  }
  if (!(rc == ONEWIRE_OK || rc == ONEWIRE_LAST_CODE)) {
    return -1;     // error
  } else {
    return i;    // number of found ROMs
  }
}

int main() {
    Scoped<DisbaleInterrupt<>> di;
    terminal::init();
    while(true) {
        static uint8_t counter = 0;
        std::cout << "c: " << ++counter << std::endl;
        
        
//        int8_t rc = scan_bus(roms, ONEWIRE_SEARCH_ROM);
      
        
        
        uint8_t n = oneWireMaster::findDevices(dsIds);
        std::cout << "n: " << n << std::endl;
        for(const auto& id : dsIds) {
            std::cout << id << std::endl;
        }
        Util::delay(1000_ms);
    }
}
#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif
