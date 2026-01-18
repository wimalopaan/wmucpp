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

#define EEPROM_MAGIC 42

struct EEProm {
    constexpr EEProm() {
        etl::copy("Output 0", outputs[0].name);
        etl::copy("Output 1", outputs[1].name);
        etl::copy("Output 2", outputs[2].name);
        etl::copy("Output 3", outputs[3].name);
        etl::copy("Output 4", outputs[4].name);
        etl::copy("Output 5", outputs[5].name);
        etl::copy("Output 6", outputs[6].name);
        etl::copy("Output 7", outputs[7].name);
#ifdef USE_MORSE
        etl::copy("SOS", morse_text);
#endif
    }
    struct Output {
        uint8_t pwm = 128;
        uint8_t iref = 128;
        uint8_t group = 0;
        uint8_t groupStart = 0;
        uint8_t control = 1;
        uint8_t slaveEnable = 0;
        std::array<char, 16> name;
    };
    struct Group {
        uint8_t iref = 128;
        uint8_t ramp = 0;
        uint8_t rate = 0;
        uint8_t stepTime = 0;
        uint8_t hold = 0;
        uint8_t holdOnTime = 1;
        uint8_t holdOffTime = 1;
        uint8_t mode = 0;
    };
    struct Virtual {
        std::array<uint8_t, 4> member{uint8_t(-1), uint8_t(-1), uint8_t(-1), uint8_t(-1)};
    };

    uint8_t magic = EEPROM_MAGIC;

    uint8_t address1 = 0; // switches 0 - 7
    uint8_t address2 = 1; // switches 8 - 15
    uint8_t address3 = 2; // groups (whatever that will mean in the future)
    uint8_t address4 = 3; // virtual switches

    uint8_t master = 0; // slave mode master

    uint8_t use_exp = 0;
    uint8_t use_virtuals = 0;

#ifdef USE_EEPROM_TEST
    uint8_t crsf_address = 0xcf;
    uint8_t response_slot = 16; // formula?
#else
    uint8_t crsf_address = 0xc8;
    uint8_t response_slot = 8;
#endif
#ifndef HW_LED2
    uint8_t telemetry = 1;
#endif

	uint8_t temp_id = 1;	
	uint8_t cells_id = 1;
	
    std::array<Output, 16> outputs{};
    std::array<Group,  4> groups{};
    std::array<Virtual, 8> virtuals{};
};
static_assert(sizeof(EEProm) < 2048);

