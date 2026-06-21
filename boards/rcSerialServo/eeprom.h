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

#include <cstdint>
#include <array>

#include "rc/rc_2.h"

#define EEPROM_MAGIC 42

struct EEProm {
    uint32_t magic = EEPROM_MAGIC;
    uint8_t address = 0xc1;
    uint8_t tempOffTelemetry = 30; // seconds for temporary telemetry off after parameter request
    
    struct ServoSetting {
        uint8_t torqueLimit = 100;
        uint8_t speed       = 100;
        uint8_t gear        = 10;
        uint8_t mode        = 0;
    };
    std::array<ServoSetting, 8> servos{};
};
