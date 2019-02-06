/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2019 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include <array>
#include <utility>

#include <etl/types.h>

#pragma pack(push)
#pragma pack(1)

namespace Hott {
    struct SumDMsg {
        inline static constexpr uint16_t ExtendedLow = 0x1c20; // 7200
        inline static constexpr uint16_t Low = 0x2260; // 8800
        inline static constexpr uint16_t Mid = 0x2ee0; // 12000
        inline static constexpr uint16_t High = 0x3b60; // 15200
        inline static constexpr uint16_t ExtendedHigh = 0x41a0; // 16800
        
        inline static constexpr uint8_t ExtendedLow8Bit = ExtendedLow >> 8; // 28
        inline static constexpr uint8_t Low8Bit = Low >> 8; // 34
        inline static constexpr uint8_t Mid8Bit = Mid >> 8; // 46
        inline static constexpr uint8_t High8Bit = High >> 8; // 59
        inline static constexpr uint8_t ExtendedHigh8Bit = ExtendedHigh >> 8; // 65
        
        inline static constexpr const uint8_t MaxChannels = 32;
        
        uint8_t nChannels = 0;
        std::array<std::pair<uint8_t, uint8_t>, MaxChannels> channelData = {};
        uint16_t crc = 0;
    };
    
}

#pragma pack(pop)
