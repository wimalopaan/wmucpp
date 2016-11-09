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

#include "config.h"
#include "mcu/ports.h"
#include "std/array.h"
#include "std/iterator.h"
#include "mcu/avr/delay.h"

template<typename DataPin, typename ClockPin, typename CSPin, bool useDelay = false>
class SoftSpiMaster final {
public:
    SoftSpiMaster() = delete;
    template<uint32_t Baud = 0>
    static void init() {
        static_assert(Baud == 0, "SoftSpiMaster doesn't use baud template parameter");
        DataPin::template dir<AVR::Output>();
        ClockPin::template dir<AVR::Output>();
        CSPin::template dir<AVR::Output>();
        CSPin::high();
        ClockPin::low();
        DataPin::low();
    }
    static bool put(uint8_t value) {
        if (useDelay) Util::delay(Config::SoftSpiMaster::pulseDelay);
        CSPin::low();
        for(uint8_t i = 0; i < 8; ++i) {
            if (useDelay) Util::delay(Config::SoftSpiMaster::pulseDelay);
            ClockPin::low();
            if (value & 0x80) {
                DataPin::high();
            }
            else {
                DataPin::low();
            }
            if (useDelay) Util::delay(Config::SoftSpiMaster::pulseDelay);
            ClockPin::high();
            value <<= 1; // shift next bit
        }
        ClockPin::low();
        if (useDelay) Util::delay(Config::SoftSpiMaster::pulseDelay);
        CSPin::high();
        DataPin::low();
        return true;
    }
private:
};
