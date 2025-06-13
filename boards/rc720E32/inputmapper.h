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

#include <cstdint>

#include "output.h"

template<typename Config>
struct InputMapper {
    using debug = Config::debug;
    using stream1 = Config::stream1;
    using stream2 = Config::stream2;
    using stream3 = Config::stream3;
    using stream4 = Config::stream4;
    using stream5 = Config::stream5;

    static inline constexpr uint16_t mid = stream1::mid;
    static inline constexpr uint16_t amp = stream1::amp;

    static inline uint16_t value(const uint8_t ch) {
        switch(mStream) {
        case 0: // Main CRSF
            return stream1::value(ch);
            break;
        case 1: // Relay-Port
            return stream2::value(ch);
            break;
        case 2: // Aux-Port
            if constexpr(!std::is_same_v<stream3, void>) {
                return stream3::value(ch);
            }
            else {
                return mid;
            }
            break;
        case 3: // BT
            if (stream4::isActive()) {
                return stream4::value(ch);
            }
            else if (stream5::isActive()) {
                return stream5::value(ch);
            }
            else {
                return mid;
            }
            break;
        default:
            return mid;
            break;
        }
    }
    static inline void stream(const uint8_t s) {
        IO::outl<debug>("Input Mapper: ", s);
        mStream = s;
    }
    private:
    static inline uint8_t mStream = 0;
};
