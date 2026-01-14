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

#include "etl/algorithm.h"

#include "pattern.h"

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
        uint8_t pwm = 0;
        uint8_t pwmDuty = 1;
        uint8_t pwmScale = 0;
        uint8_t blink = 0;
        uint8_t blinkOnTime = 1;
        uint8_t blinkOffTime = 2;
        uint8_t flashCount = 1;
#ifdef USE_SLAVE_COMMAND
        uint8_t slaveEnable = 0;
#endif
        std::array<char, 16> name;
    };

    uint8_t magic = EEPROM_MAGIC;

    uint8_t pwm1 = 10;
    uint8_t pwm2 = 10;
    uint8_t pwm3 = 10;
    uint8_t pwm4 = 10;

#ifdef USE_VIRTUALS
    struct AdrIndex {
        static inline constexpr uint8_t Switch  = 0;
        static inline constexpr uint8_t Virtual = 1;
    };
    std::array<uint8_t, 2> addresses = {0, 1};
#else
    struct AdrIndex {
        static inline constexpr uint8_t Switch  = 0;
    };
    std::array<uint8_t, 1> addresses = {0};
#endif

    uint8_t crsf_address = 0xc8;

#ifdef USE_RESPONSE_SLOT
    uint8_t response_slot = 8;
#else
    uint8_t response_slot = 0;
#endif
#ifdef USE_TELEMETRY
    uint8_t telemetry = 1;
#else
    uint8_t telemetry = 0;
#endif
#if defined(HW_MSW11) || defined(HW_MSW12)
    uint8_t cells_id = 1;
    uint8_t temp_id = 1;
#endif
#ifdef USE_VIRTUALS
    uint8_t use_virtuals = 1;
#endif
    std::array<Output, 8> outputs{};
#ifdef USE_MORSE
    std::array<char, 64> morse_text{};
    uint8_t morse_dit = 3;
    uint8_t morse_dah = 6;
    uint8_t morse_gap = 3;
    uint8_t morse_igap = 3;
#endif

#ifdef USE_VIRTUALS
    struct Virtual {
        std::array<uint8_t, 4> member{};
    };
    std::array<Virtual, 4> virtuals{};
#endif
#ifdef USE_PATTERNS
	struct Pattern {
		uint8_t type = 0;
		std::array<uint8_t, 8> member{1, 2, 3, 4, 5, 6, 7, 8};
		uint8_t onTime = 33; // time 10 ms
		uint8_t offTime = 1;
		uint8_t next_address = 3; // next multiswitch's virtual address
		uint8_t group = 1; 
	};
    std::array<External::Pattern::EEProm, 4> pattern{};
#endif

#ifdef USE_SLAVE_COMMAND
    uint8_t slaveSend = 0;
    uint8_t master = 0;
#endif

#ifdef USE_FAILSAFE
    uint8_t failsafe_mode    = 0; // 0: hold; 1: all_off, 2: set
    std::array<uint8_t, 8> failsafe_pattern = {0}; // on/off if mode == 2
#endif
};
static_assert(sizeof(EEProm) < 2048);
