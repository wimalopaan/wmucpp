/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <array>
#include "crc.h"

template<typename Device>
struct MessageBuilder {
    using device = Device;
    explicit MessageBuilder(std::array<std::byte, Crsf::maxMessageSize>& buffer,
                            const std::byte type) : mBuffer(buffer) {
        mBuffer[0] = 0xc8_B;
        mLength = 3;
        mBuffer[1] = std::byte(mLength);
        mBuffer[2] = type;

    }
    ~MessageBuilder() {
        const uint8_t length = mLength + 1; // incl. CRC
        mBuffer[1] = std::byte(length - 2); // without: startbyte and length, including CRC
        CRC8 csum;
        for(int8_t i = 0; i < length - 2 - 1; ++i) { // without CRC
            csum += mBuffer[i + 2];
        }
        mBuffer[length - 1] = csum;
        for(uint8_t i = 0; i < length; ++i) {
            if constexpr(!std::is_same_v<device, void>) {
                device::put(mBuffer[i]);
            }
        }
    }
    void push_back(const uint8_t d) {
        mBuffer[mLength++] = std::byte(d);
    }
    void push_back(const std::byte d) {
        mBuffer[mLength++] = d;
    }
    void push_back(const uint16_t d) {
        mBuffer[mLength++] = std::byte(d >> 8);
        mBuffer[mLength++] = std::byte(d);
    }
    void push_back(const int16_t d) {
        mBuffer[mLength++] = std::byte(d >> 8);
        mBuffer[mLength++] = std::byte(d);
    }
    private:
    uint8_t mLength{};
    std::array<std::byte, Crsf::maxMessageSize>& mBuffer;
};

