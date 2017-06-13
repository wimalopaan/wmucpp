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

#include <stdlib.h>

#include <string.h>

#include "mcu/ports.h"
#include "external/ds18b20.h"
#include "external/onewire.h"
#include "hal/softspimaster.h"
#include "console.h"

#include "ow.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

using terminal = SSpi0;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

using oneWirePin = AVR::Pin<PortA, 5>;
using oneWireMaster = OneWire::Master<oneWirePin, OneWire::Normal, false, false>;

std::array<OneWire::ow_rom_t, 5> dsIds;

#define MAX_ROMS 10
typedef uint8_t rom_t[MAX_ROMS][8];
uint8_t buffer[9] = {0x10,  0x1D, 0x9E, 0x09, 0x02, 0x08, 0x00, 0x55, 0x00};  // init values just for test & debugging
rom_t roms;

int scan_bus(rom_t rom_list, uint8_t cmd) {
  uint8_t i = 0, rc = 0;

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
      
        
        
        oneWireMaster::findDevices(dsIds);
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
