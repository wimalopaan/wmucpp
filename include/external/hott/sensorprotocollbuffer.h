/*k
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

#include <cstdint>
#include <optional>

#include "units/duration.h"
#include "units/physical.h"
#include "util/fixedpoint.h"
#include "sensorprotocoll.h"

namespace Hott {
    
    template<uint8_t N>
    class SensorProtocollBuffer final {
    public:
        typedef uint8_t index_type;
        static constexpr const uint8_t number = N;
        static constexpr const uint8_t cyclesBeforeAnswer = 1 + Hott::hottDelayBeforeAnswer / Hott::hottDelayBetweenBytes;
        
        SensorProtocollBuffer() = delete;

        inline static std::optional<std::byte> get(uint8_t index) {
            if (index < cyclesBeforeAnswer) {
                return {};
            }
            else {
                index -= cyclesBeforeAnswer;
                if (index < sizeof(hottBinaryResponse)) {
                    return getByte(index);
                }
                return {};
            }
        }
        inline static constexpr uint8_t size() {
            return sizeof(hottBinaryResponse) + cyclesBeforeAnswer;
        }
        inline static constexpr void reset() {
            hottBinaryResponse.parity = 0;
        }
        inline static constexpr void init() {
            hottBinaryResponse.start_byte = 0x7c;
            hottBinaryResponse.gam_sensor_id = 0x8d;
            hottBinaryResponse.sensor_id = 0xd0;
            hottBinaryResponse.stop_byte = 0x7d;
        }
        
        inline static void rpm1(const std::RPM& v) {
            if (v) {
                hottBinaryResponse.rpm = v.value() / 10;
            }
        }
        inline static std::RPM rpm1() {
            return std::RPM{hottBinaryResponse.rpm * 10};
        }
        inline static void rpm2(const std::RPM& v) {
            if (v) {
                hottBinaryResponse.rpm2 = v.value() / 10;
            }
        }
        inline static std::RPM rpm2() {
            return std::RPM{hottBinaryResponse.rpm2 * 10};
        }
        
        inline static void temp1(FixedPoint<int, 4> v) {
            hottBinaryResponse.temperature1 = v.integer() + 20;
        }
        inline static void temp2(FixedPoint<int, 4> v) {
            hottBinaryResponse.temperature2 = v.integer() + 20;
        }
        
        inline static void temperatureRaw(uint8_t i, uint8_t v) {
            assert(i < 2);
            if (i == 0) {
                hottBinaryResponse.temperature1 = v;
            }
            else {
                hottBinaryResponse.temperature2 = v;
            }
        }

        inline static void cellVoltageRaw(uint8_t cell, uint8_t v) {
            hottBinaryResponse.cell[cell] = v;
        } 
        inline static void batteryVoltageRaw(uint8_t battery, uint16_t v) {
            assert(battery < 2);
            if (battery == 0) {
                hottBinaryResponse.Battery1 = v;
            }
            else {
                hottBinaryResponse.Battery2 = v;
            }
        }
        
        inline static void batteryMinimumRaw(uint8_t cell, uint16_t c) {
            hottBinaryResponse.min_cell_volt_num = cell;
            hottBinaryResponse.min_cell_volt = c;
        }
        inline static void currentRaw(uint16_t v) {
            hottBinaryResponse.current = v;
        }

    private:
        inline static std::byte getByte(uint8_t index) {
            assert(index < sizeof(hottBinaryResponse));
            constexpr const std::byte* ptr = (const std::byte*) &hottBinaryResponse;  
            const std::byte value = ptr[index];
            hottBinaryResponse.parity += std::to_integer<uint8_t>(value);
            return value;    
        }
        
        inline static GamMsg hottBinaryResponse{}; 
        static_assert((cyclesBeforeAnswer + sizeof(hottBinaryResponse)) < std::numeric_limits<uint8_t>::max());
        
    };
}
