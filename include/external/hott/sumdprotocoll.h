/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

#pragma pack(push)
#pragma pack(1)

namespace Hott {

struct SumDMsg {
    static constexpr uint16_t ExtendedLow = 0x1c20;
    static constexpr uint16_t Low = 0x2260;
    static constexpr uint16_t Mid = 0x2ee0;
    static constexpr uint16_t High = 0x3b60;
    static constexpr uint16_t ExtendedHigh = 0x41a0;

    static constexpr uint8_t ExtendedLow8Bit = ExtendedLow >> 8;
    static constexpr uint8_t Low8Bit = Low >> 8;
    static constexpr uint8_t Mid8Bit = Mid >> 8;
    static constexpr uint8_t High8Bit = High >> 8;
    static constexpr uint8_t ExtendedHigh8Bit = ExtendedHigh >> 8;

    static constexpr const uint8_t MaxChannels = 32;
    volatile uint8_t  nChannels = 0;
    volatile std::array<std::pair<uint8_t>, MaxChannels> channelData = {};
    volatile uint16_t crc = 0;
};

}

#pragma pack(pop)
