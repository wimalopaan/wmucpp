/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include "util/dassert.h"
#include "mcu/avr/twimaster.h"

// todo: set time from DateTime::TimeTM

template<typename TWIMaster>
class DS1307 : public EventHandler<EventType::TWIRecvComplete> {
public:
    static constexpr TWI::Address Address{0x68};
    static constexpr const std::hertz fSCL = 100000_Hz;
    
    static void init() {
        TWIMaster::template init<fSCL>();
    }
    template<bool On>
    static void halt() {
        static_assert(!TWIMaster::isAsync, "use only with synchron TWI Master");
        std::array<uint8_t, 2> data;
        data[0] = 0x00;
        data[1] = On ? 0x80 : 0x00;
        TWIMaster::template write<Address>(data);
    }

    template<bool On>
    static bool squareWave() {
        std::array<uint8_t, 2> data;
        data[0] = 0x07;
        data[1] = On ? 0x10 : 0x80;
        if constexpr(TWIMaster::isAsync) {
            return TWIMaster::template startWrite<Address>(data);
        }
        else {
            return TWIMaster::template write<Address>(data);
        }
    }

    template<uint8_t Pointer, uint8_t Length>
    static void startRead() {
        static_assert(TWIMaster::isAsync, "use only with asynch TWI Master");
        static_assert(Pointer < 64, "wrong pointer");
        static_assert((Pointer + Length) <= 64, "wrong pointer or length");
        TWIMaster::template startReadWithPointer<Address, Pointer, Length>();
    }
    
    static std::optional<uint8_t> readControlRegister() {
        static_assert(!TWIMaster::isAsync, "use only with synchron TWI Master");
        std::array<uint8_t, 1> data;
        if (!TWIMaster::template readWithPointer<Address, 0x07>(data)) {
            return {};
        }        
        return data[0];
    }

    static bool startWrite(uint8_t index, uint8_t value) {
        static_assert(TWIMaster::isAsync, "use only with asynch TWI Master");
        assert(index < 56);
        std::array<uint8_t, 2> data;
        data[0] = 0x08 + index;
        data[1] = value;
        return TWIMaster::template startWrite<Address>(data);        
    }
    
    static bool startRead(uint8_t index, uint8_t number) {
        static_assert(TWIMaster::isAsync, "use only with asynch TWI Master");
        assert(index < 64);
        assert((index + number) <= 64);
        return TWIMaster::template startReadWithPointer<Address>(index, number);        
    }
    
    static bool writeToRam(uint8_t index, uint8_t value) {
        assert(index < 56);
        std::array<uint8_t, 2> data;
        data[0] = 0x08 + index;
        data[1] = value;
        return TWIMaster::template write<Address>(data);        
    }
    
    static std::optional<uint8_t> readFromRam(uint8_t index) {
        assert(index < 56);
        std::array<uint8_t, 1> data;
        if (!TWIMaster::template readWithPointer<Address>(data, 0x08 + index)) {
            return {};
        }        
        return data[0];
    }

    template<uint8_t Index>
    static std::optional<uint8_t> readFromRam() {
        static_assert(Index < 56);
        std::array<uint8_t, 1> data;
        if (!TWIMaster::template readWithPointer<Address, Index>(data)) {
            return {};
        }        
        return data[0];
    }
    
    static bool readTimeInfo() {
        return TWIMaster::template readWithPointer<Address, 0>(mTimeInfo);
    }

    static bool startReadTimeInfo() {
        return TWIMaster::template startReadWithPointer<Address, 0, mTimeInfo.size>();
    }
    
    static const std::array<uint8_t, 7>& timeInfo() {
        return mTimeInfo;
    }

    static void process(uint8_t address) {
        if (TWI::Address{address} == Address) {
            for(uint8_t i = 0; i < mTimeInfo.size; ++i) {
                if (auto v = TWIMaster::get()) {
                    mTimeInfo[i] = *v;
                }
                else {
                    EventManager::enqueue({EventType::DS1307Error, 0});
                    return;
                }
            }
            EventManager::enqueue({EventType::DS1307TimeAvailable, 0});
        }
    }

private:
    static std::array<uint8_t, 7> mTimeInfo;   
};
template<typename TWIMaster>
std::array<uint8_t, 7> DS1307<TWIMaster>::mTimeInfo;