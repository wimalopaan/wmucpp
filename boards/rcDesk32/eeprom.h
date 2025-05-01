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

enum class Stream : uint8_t {Trainer, VControls, Off};

struct Map {
    Stream stream = Stream::Off; // trainer;vcontrols;off
    uint8_t position = 0;
    uint8_t count = 0;
};

struct EEProm {
    consteval EEProm() = default;

    uint8_t aux1mode = 1; // sbus;inverted-sbus;hwext;off
    uint8_t aux2mode = 2; // sbus;inverted-sbus;hwext;off
    uint8_t busmode = 0;
    uint8_t sm1mode = 0; // sm;off
    uint8_t sm2mode = 0; // sm;off

    std::array<Map, 2> analogMaps{
        Map{Stream::VControls, 0, 3},
        Map{Stream::VControls, 3, 3}
    };
    std::array<Map, 2> smMaps{
        Map{Stream::Trainer, 0, 6},
        Map{Stream::Trainer, 7, 6}
    };
    std::array<Map, 2> incMaps{
        Map{Stream::VControls, 6, 1},
        Map{Stream::VControls, 7, 1}
    };

    uint8_t bluetooth = 1;
    Map bluetoothMap{Stream::VControls, 8, 4};

    uint8_t crsf_in = 0;
    Map crsfInMap{Stream::VControls, 12, 4};

    uint8_t mode = 0;
    uint8_t prop8mode = 0; // send prop values as 8-bit
    uint8_t address = 0xc8;
    uint8_t controllerNumber = 0;
};
