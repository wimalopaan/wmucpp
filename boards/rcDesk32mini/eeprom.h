/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#define EEPROM_MAGIC 42

struct EEProm {
    uint8_t magic = EEPROM_MAGIC;

	struct CalibValues {
		uint16_t min = 0;
		uint16_t max = 4095;
		uint16_t mid = 2048;
		uint16_t span = max - min;
	};
	
	std::array<CalibValues, 8> calibration{};
	
	uint8_t controllerNumber = 0;
	uint8_t prop8mode = 0;
};
