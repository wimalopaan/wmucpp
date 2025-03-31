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
#include <array>

template<uint8_t Size = 3>
struct Map {
    uint8_t stream = 0; // trainer; vcontrols;off
    uint8_t position = 0;
    // std::array<bool, Size> enable{};
};
template<>
struct Map<0> {
    uint8_t stream = 0; // trainer; vcontrols;off
    uint8_t position = 0;
};

struct EEProm {
    consteval EEProm() = default;

    uint8_t aux1mode = 1; // sbus;inverted-sbus;hwext;off
    uint8_t aux2mode = 2; // sbus;inverted-sbus;hwext;off
    uint8_t busmode = 0;
    uint8_t sm1mode = 0; // sm;off
    uint8_t sm2mode = 0; // sm;off

    std::array<Map<3>, 2> analogMaps{
        Map<3>{1, 0},
        Map<3>{1, 3}
    };
    std::array<Map<6>, 2> smMaps{
        Map<6>{0, 0},
        Map<6>{0, 7}
    };
    std::array<Map<0>, 2> incMaps{
        Map<0>{1, 6},
        Map<0>{1, 7}
    };

    Map<0> bluetoothMap;
    Map<0> crsfInMap;

    uint8_t mode = 0;
    uint8_t prop8mode = 0; // send prop values as 8-bit
    uint8_t address = 0xc8;
    uint8_t controllerNumber = 1;
    uint8_t bluetooth = 1;
    uint8_t crsf_in = 1;
};
