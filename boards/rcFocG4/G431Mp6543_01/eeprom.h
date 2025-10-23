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

#define EEPROM_MAGIC 42

struct EEProm {
    uint32_t magic = EEPROM_MAGIC;
	
	uint8_t crsf_address = 0xc8;
	uint8_t switch_address = 0;
	uint8_t response_slot = 0;
	
	uint8_t mode = 0;
	
	struct ServoParams {
		uint8_t max_throw = 100;
		uint8_t max_speed = 100;
		uint8_t max_force = 100;
		uint8_t auto_stop = 0;
	};

	std::array<ServoParams, 4> mode_params{};
	
};
static_assert(sizeof(EEProm) < 2048);

