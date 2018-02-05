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

#include "std/array"
#include "mcu/avr/twimaster.h"
#include "hal/event.h"
#include "util/dassert.h"

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


namespace I2C {
    namespace detail {
        struct Base {
            static constexpr EventType eventType = EventType::NoEvent;
        };
    }
}

template<typename TWIMaster, const TWI::Address& Address, typename Parameter = I2CRamParameter, ::Util::NamedFlag useEvents = UseEvents<true>>
class I2CGeneric : public std::conditional<useEvents::value, EventHandler<EventType::TWIRecvComplete>, I2C::detail::Base>::type {
public:
    template<const std::hertz& fSCL>
    static void init() {
        TWIMaster::template init<fSCL>();
    }
    
    static bool write(uint8_t index, std::byte value) {
        std::array<std::byte, 2> data = {std::byte{index}, value};
        return TWIMaster::template write<Address>(data);
    }
    static std::optional<std::byte> read(uint8_t index) {
        std::array<std::byte, 1> data;
        if (TWIMaster::template readWithPointer<Address>(data, index)) {
            return data[0];
        }
        return {};
    }
    
    static bool startRead(uint8_t index) {
        return TWIMaster::template startReadWithPointer<Address>(TWI::Range{index, 1});
    }

    static bool startWrite(uint8_t index, std::byte value) {
        std::array<std::byte, 2> data;
        data[0] = std::byte{index};
        data[1] = value;
        return TWIMaster::template startWrite<Address>(data);        
    }
    
    template<bool Q = useEvents::value>
    inline static 
    typename std::enable_if<Q, bool>::type
    process(std::byte b) {
        auto address = b;
        if (TWI::Address{address} == Address) {
            if (auto v = TWIMaster::get()) {
                EventManager::enqueue({Parameter::eventValueAvailable, std::byte{*v}});
            }
            else {
                EventManager::enqueue({Parameter::eventError});
            }
            return true;
        }
        return false;
    }
private:
};

