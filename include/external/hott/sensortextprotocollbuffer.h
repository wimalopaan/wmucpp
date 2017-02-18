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

#include "std/optional.h"
#include "units/duration.h"
#include "sensorprotocoll.h"

namespace Hott {

template<uint8_t N>
class SensorTextProtocollBuffer {
    SensorTextProtocollBuffer() = delete;
public:
    static constexpr const uint8_t number = N;
    static constexpr const uint8_t cyclesBeforeAnswer = Hott::hottDelayBeforeAnswer / Hott::hottDelayBetweenBytes;

    static std::optional<uint8_t> get(uint8_t index) {
        if (index < cyclesBeforeAnswer) {
            return {};
        }
        else {
            index -= cyclesBeforeAnswer;
            if (index < sizeof(hottTextResponse)) {
                constexpr const uint8_t* ptr = (const uint8_t*) &hottTextResponse;  
                const uint8_t value = ptr[index];
                hottTextResponse.parity += value;
                return value;    
            }
            else {
                return {};
            }
        }
    }
    static constexpr uint8_t size() {
        return sizeof(hottTextResponse) + cyclesBeforeAnswer;
    }
    static constexpr void reset() {
        hottTextResponse.parity = 0;
    }
    static constexpr void init() {
        hottTextResponse.start_byte = 0x7b;
        hottTextResponse.stop_byte = 0x7d;
        hottTextResponse.esc = 0;
        
        hottTextResponse.text[0].insertAtFill(0, " Test1"_pgm);
        hottTextResponse.text[1].insertAtFill(0, " Test2"_pgm);
        hottTextResponse.text[2].insertAtFill(0, " Test3"_pgm);
        hottTextResponse.text[3].insertAtFill(0, " Test4"_pgm);
        hottTextResponse.text[4].insertAtFill(0, " Test5"_pgm);
        hottTextResponse.text[5].insertAtFill(0, " Test6"_pgm);
        hottTextResponse.text[6].insertAtFill(0, " Test7"_pgm);
        hottTextResponse.text[7].insertAtFill(0, " Test8"_pgm);
    }
    static auto& text() {
        return hottTextResponse.text;
    }

private:
    static TextMsg hottTextResponse;
};
template<uint8_t N>
TextMsg SensorTextProtocollBuffer<N>::hottTextResponse;

}
