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

#include "std/array.h"
#include "mcu/avr/twimaster.h"
#include "hal/event.h"
#include "util/dassert.h"

// todo: umbenennen: I2CGeneric

struct I2CRamParameter {
    static constexpr EventType eventValueAvailable = EventType::I2CRamValueAvailable;
    static constexpr EventType eventError = EventType::I2CRamError;
};

struct I2CLedParameter {
    static constexpr EventType eventValueAvailable = EventType::I2CLedValueAvailable;
    static constexpr EventType eventError = EventType::I2CLedError;
};

struct I2CRpmParameter {
    static constexpr EventType eventValueAvailable = EventType::I2CRpmValueAvailable;
    static constexpr EventType eventError = EventType::I2CRpmError;
};

template<typename TWIMaster, const TWI::Address& Address, typename Parameter = I2CRamParameter>
class I2CRam : public EventHandler<EventType::TWIRecvComplete> {
public:
    template<const std::hertz& fSCL>
    static void init() {
        TWIMaster::template init<fSCL>();
    }
    
    static bool write(uint8_t index, uint8_t value) {
        std::array<uint8_t, 2> data = {index, value};
        return TWIMaster::template write<Address>(data);
    }
    static std::optional<uint8_t> read(uint8_t index) {
        std::array<uint8_t, 1> data;
        if (TWIMaster::template readWithPointer<Address>(data, index)) {
            return data[0];
        }
        return {};
    }
    
    static bool startRead(uint8_t index) {
        return TWIMaster::template startReadWithPointer<Address>(TWI::Range{index, 1});
    }

    static bool startWrite(uint8_t index, uint8_t value) {
        std::array<uint8_t, 2> data;
        data[0] = index;
        data[1] = value;
        return TWIMaster::template startWrite<Address>(data);        
    }
    
    static void process(uint8_t address) {
        if (TWI::Address{address} == Address) {
            if (auto v = TWIMaster::get()) {
                EventManager::enqueue({Parameter::eventValueAvailable, *v});
            }
            else {
                EventManager::enqueue({Parameter::eventError, 0});
            }
        }
    }
private:
};

