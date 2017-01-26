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
#include "hal/event.h"
#include "sensorprotocoll.h"

namespace Hott {
    
template<uint8_t N>
struct NullPA {
    inline static bool process(uint8_t) { // from isr only
        static uint8_t counter = 0;
        if (++counter > 100) {
            EventManager::enqueue({EventType::NullPAEvent, N});
            counter = 0;
        }
        return true;
    }    
};

template<uint8_t M>
class SensorProtocollAdapter final {
    enum class hottstate {Undefined = 0, Request1, RequestA1, NumberOfStates};
    template<int N, typename PA> friend class AVR::Usart;
public:
    SensorProtocollAdapter() = delete;
private:
    inline static bool process(uint8_t c) { // from isr only
        static hottstate state = hottstate::Undefined;
        switch (state) {
        case hottstate::Undefined:
            if (c == 0x80) {
                state = hottstate::Request1;
            }
            if (c == 0x7f) {
                state = hottstate::RequestA1;
            }
            break;
        case hottstate::RequestA1:
            if (c == 0x0f) {
                EventManager::enqueueISR({EventType::HottAsciiRequest, 0});
                state = hottstate::Undefined;
            }
            else {
                EventManager::enqueueISR({EventType::HottAsciiKey, c});
                state = hottstate::Undefined;
            }
            break;
        case hottstate::Request1:
            if (c == 0x8d) {
                EventManager::enqueueISR({EventType::HottBinaryRequest, 0});
                state = hottstate::Undefined;
            }
            else if (c == 0x80) {
                EventManager::enqueueISR({EventType::HottSensorBroadcast, 0});
                state = hottstate::Undefined;
            }
            else {
                state = hottstate::Undefined;
            }
            break;
        default:
            assert(false);
            break;
        }
        return true;
    }
};

}