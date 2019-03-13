/*k
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

#pragma once

#include <cstdint>
#include <optional>
#include <etl/fixedpoint.h>

#include "sensorprotocoll.h"

namespace Hott {
    using namespace etl;
    using namespace External::Units;
    
    template<uint8_t N>
    class SensorProtocollBuffer final {
        SensorProtocollBuffer() = delete;
        inline static GamMsg hottBinaryResponse{}; 
    public:
        typedef uint8_t index_type;

        static constexpr const uint8_t number = N;
        static constexpr const uint8_t cyclesBeforeAnswer = 1 + Hott::hottDelayBeforeAnswer / Hott::hottDelayBetweenBytes;

        static_assert((cyclesBeforeAnswer + sizeof(hottBinaryResponse)) < std::numeric_limits<index_type>::max());

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
        }
        
        inline static void rpm1(const RPM& v) {
            if (v) {
                hottBinaryResponse.rpm = v.value() / 10;
            }
        }
        inline static RPM rpm1() {
            return RPM{hottBinaryResponse.rpm * 10};
        }
        inline static void rpm2(const RPM& v) {
            if (v) {
                hottBinaryResponse.rpm2 = v.value() / 10;
            }
        }
        inline static RPM rpm2() {
            return RPM{hottBinaryResponse.rpm2 * 10};
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
        inline static void mainVoltageRaw(uint16_t v) {
            hottBinaryResponse.main_voltage = v;
        }
        inline static void speedRaw(uint16_t v) {
            hottBinaryResponse.speed = v;
        }
        inline static void forceRaw(uint16_t v) {
            hottBinaryResponse.climbrate_L = v;
            hottBinaryResponse.fuel_ml = v;
        }
    private:
        inline static std::byte getByte(uint8_t index) {
            assert(index < sizeof(hottBinaryResponse));
            /*constexpr */const std::byte* ptr = (const std::byte*) &hottBinaryResponse;  
            const std::byte value = ptr[index];
            hottBinaryResponse.parity += std::to_integer<uint8_t>(value);
            return value;    
        }
        
    };

    template<uint8_t N>
    class EscProtocollBuffer final {
        EscProtocollBuffer() = delete;
        inline static EscMsg EscResponse{}; 
    public:
        typedef uint8_t index_type;

        static constexpr const uint8_t number = N;
        static constexpr const uint8_t cyclesBeforeAnswer = 1 + Hott::hottDelayBeforeAnswer / Hott::hottDelayBetweenBytes;

        static_assert((cyclesBeforeAnswer + sizeof(EscResponse)) < std::numeric_limits<index_type>::max());

        inline static std::optional<std::byte> get(uint8_t index) {
            if (index < cyclesBeforeAnswer) {
                return {};
            }
            else {
                index -= cyclesBeforeAnswer;
                if (index < sizeof(EscResponse)) {
                    return getByte(index);
                }
                return {};
            }
        }
        inline static constexpr uint8_t size() {
            return sizeof(EscResponse) + cyclesBeforeAnswer;
        }
        inline static constexpr void reset() {
            EscResponse.parity = 0;
        }
        inline static constexpr void init() {
        }
        inline static void rpmRaw(uint16_t v) {
            EscResponse.rpm= v;
        }
        inline static void rpmMaxRaw(uint16_t v) {
            EscResponse.rpm_max = v;
        }
        inline static void voltageRaw(uint16_t v) {
            EscResponse.voltage = v;
        }
        inline static void voltageMinRaw(uint16_t v) {
            EscResponse.voltage_min = v;
        }
        inline static void currentRaw(uint16_t v) {
            EscResponse.current = v;
        }
        inline static void currentMaxRaw(uint16_t v) {
            EscResponse.current_max = v;
        }
        inline static void tempRaw(uint16_t v) {
            EscResponse.temp = v;
        }
        inline static void tempMaxRaw(uint16_t v) {
            EscResponse.temp_max = v;
        }
        
        inline static void forceRaw(uint16_t v) {
        }
    private:
        inline static std::byte getByte(uint8_t index) {
            assert(index < sizeof(hottBinaryResponse));
            /*constexpr */const std::byte* ptr = (const std::byte*) &EscResponse;  
            const std::byte value = ptr[index];
            EscResponse.parity += std::to_integer<uint8_t>(value);
            return value;    
        }
        
    };
}
