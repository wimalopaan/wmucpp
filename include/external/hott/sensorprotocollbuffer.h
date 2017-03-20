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
#include "units/physical.h"
#include "sensorprotocoll.h"

namespace Hott {

template<uint8_t N>
class SensorProtocollBuffer {
public:
    static constexpr const uint8_t number = N;
    static constexpr const uint8_t cyclesBeforeAnswer = Hott::hottDelayBeforeAnswer / Hott::hottDelayBetweenBytes;
    
    SensorProtocollBuffer() = delete;

    static std::optional<uint8_t> get(uint8_t index) {
        if (index < cyclesBeforeAnswer) {
            return {};
        }
        else {
            index -= cyclesBeforeAnswer;
            if (index < sizeof(hottBinaryResponse)) {
                constexpr const uint8_t* ptr = (const uint8_t*) &hottBinaryResponse;  
                const uint8_t value = ptr[index];
                hottBinaryResponse.parity += value;
                return value;    
            }
            else {
                return {};
            }
        }
    }
    static constexpr uint8_t size() {
        return sizeof(hottBinaryResponse) + cyclesBeforeAnswer;
    }
    static constexpr void reset() {
        hottBinaryResponse.parity = 0;
    }
    static constexpr void init() {
        hottBinaryResponse.start_byte = 0x7c;
        hottBinaryResponse.gam_sensor_id = 0x8d;
        hottBinaryResponse.sensor_id = 0xd0;
        hottBinaryResponse.cell[0] = 111;
        hottBinaryResponse.cell[1] = 222;
        hottBinaryResponse.cell[2] = 10;
        hottBinaryResponse.rpm = 111;
        hottBinaryResponse.rpm2 = 222;
        hottBinaryResponse.stop_byte = 0x7d;
    }
    
    static void rpm1(const std::RPM& v) {
        if (v) {
            hottBinaryResponse.rpm = v.value() / 10;
        }
    }
    static std::RPM rpm1() {
        return std::RPM{hottBinaryResponse.rpm * 10};
    }
    static void temp1(FixedPoint<int, 4> v) {
        hottBinaryResponse.temperature1 = v.integer() + 20;
    }
    static void temp2(FixedPoint<int, 4> v) {
        hottBinaryResponse.temperature2 = v.integer() + 20;
    }

private:
    inline static GamMsg hottBinaryResponse;
};
}
