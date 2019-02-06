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

#pragma once

#include <cstdint>
#include <optional>

#include "sensorprotocoll.h"

namespace Hott {
    
    template<uint8_t N, etl::Concepts::NamedFlag useEvents = etl::NamedFlag<true>>
    class SensorTextProtocollBuffer final {
        SensorTextProtocollBuffer() = delete;
    public:
        inline static constexpr auto rows = TextMsg::rows; 
        inline static constexpr auto columns = TextMsg::columns; 
        
        using index_type = uint8_t;
        inline static constexpr uint8_t number = N;
        inline static constexpr uint8_t cyclesBeforeAnswer = Hott::hottDelayBeforeAnswer / Hott::hottDelayBetweenBytes;
        
        inline static std::optional<std::byte> get(uint8_t index) {
            if (index < cyclesBeforeAnswer) {
                return {};
            }
            else {
                index -= cyclesBeforeAnswer;
                if (index < sizeof(hottTextResponse)) {
                    return getByte(index);
                }
                return {};
            }
        }
        inline static constexpr uint8_t size() {
            return sizeof(hottTextResponse) + cyclesBeforeAnswer;
        }
        inline static constexpr void reset() {
            hottTextResponse.parity = 0;
        }
        inline static constexpr void init() {
            hottTextResponse.esc = 0;
            for(auto& l : hottTextResponse.text) {
                l.clear();
            }
        }
        inline static auto& text() {
            return hottTextResponse.text;
        }
    private:
        inline static std::byte getByte(uint8_t index) {
            assert(index < sizeof(hottTextResponse));
            const std::byte* const ptr = reinterpret_cast<const std::byte*>(&hottTextResponse);  
            const auto value = ptr[index];
            hottTextResponse.parity += std::to_integer<uint8_t>(value);
            return value;    
        }
        inline static TextMsg hottTextResponse{};
        static_assert((cyclesBeforeAnswer + sizeof(hottTextResponse)) < std::numeric_limits<uint8_t>::max());
    };
    
}
