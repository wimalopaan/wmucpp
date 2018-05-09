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

#include <cstdint>
#include <optional>

#include "units/duration.h"
#include "units/physical.h"
#include "util/fixedpoint.h"
#include "external/hott/sensorprotocollbuffer.h"

//struct GamMsg {
//    uint8_t start_byte; 
//    uint8_t start_byte2; 
//    std::array<uint8_t, 6> cell;
//};

//template<uint8_t N>
//class SensorProtocollBuffer final {
//public:
//    inline static constexpr void init() {
//        hottBinaryResponse.cell[0] = 0;
//        hottBinaryResponse.start_byte2 = 0;
//    }
//    inline static GamMsg hottBinaryResponse{}; 
//};

using sensorData = Hott::SensorProtocollBuffer<0>;

int main() {
    sensorData::init();
}
