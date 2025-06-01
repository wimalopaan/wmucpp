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
        uint8_t blinkOffTime = 1;
        std::array<char, 16> name;
#ifdef USE_AUTO_CONF
        uint8_t ls = 0;
        uint8_t type = 0;
        uint8_t flags = 0;
#endif
    };

    uint8_t magic = EEPROM_MAGIC;

    uint8_t address = 0;
#ifdef USE_EEPROM_TEST
    uint8_t crsf_address = 0xcf;
    uint8_t response_slot = 16; // formula?
#else
    uint8_t crsf_address = 0xc8;
    uint8_t response_slot = 8;
#endif
#ifdef USE_EEPROM_TEST
    uint8_t telemetry = 1;
#else
    uint8_t telemetry = 0;
#endif
    std::array<Output, 8> outputs{};
#ifdef USE_MORSE
    std::array<char, 64> morse_text{};
    uint8_t morse_dit = 3;
    uint8_t morse_dah = 6;
    uint8_t morse_gap = 3;
    uint8_t morse_igap = 3;
#endif
};

