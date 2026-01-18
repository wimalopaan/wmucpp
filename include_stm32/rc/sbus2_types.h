/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#pragma once

#include <cstdint>
#include <cstddef>

#include "etl/ranged.h"

namespace RC::Protokoll::SBus2 {
    using namespace etl::literals;
    using namespace std::literals::chrono_literals;
    static inline constexpr uint32_t baud = 100'000;
    static inline constexpr uint8_t start_byte = 0x0f;
    static inline constexpr uint8_t end_byte = 0x00;

    static inline constexpr uint8_t flagsIndex = 23;
    static inline constexpr uint8_t endIndex = 24;

    static inline constexpr uint8_t ch17 = 0x01;
    static inline constexpr uint8_t ch18 = 0x02;
    static inline constexpr uint8_t frameLost = 0x04;
    static inline constexpr uint8_t failSafe = 0x08;

    inline static constexpr uint16_t sbus_min = 172;
    inline static constexpr uint16_t sbus_max = 1811;

    inline static constexpr uint16_t sbus_mid = (sbus_max + sbus_min) / 2;

    using value_type = etl::ranged<sbus_min, sbus_max>;
    using index_type = etl::ranged<0, 15>;
}
