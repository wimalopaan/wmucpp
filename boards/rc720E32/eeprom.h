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

#include "etl/algorithm.h"

#include <cstdint>
#include <array>

#include "rc/rc_2.h"

struct EEProm {
    using eeprom_value_t = uint16_t;

    consteval EEProm() = default;

    struct Pair {
        eeprom_value_t first;
        eeprom_value_t second;
    };
    struct Switch {
        std::array<char, 16> name;
        uint8_t ls = 0;
        uint8_t type = 0;
        uint8_t flags = 0;
    };

    uint8_t config_counter = 0;

    std::array<Switch, 8> switches = []{
        std::array<Switch, 8> sw{};
        etl::copy("Output 0", sw[0].name);
        etl::copy("Output 1", sw[1].name);
        etl::copy("Output 2", sw[2].name);
        etl::copy("Output 3", sw[3].name);
        etl::copy("Output 4", sw[4].name);
        etl::copy("Output 5", sw[5].name);
        etl::copy("Output 6", sw[6].name);
        etl::copy("Output 7", sw[7].name);
        return sw;
    }();

    // Generell
#ifdef TEST_EEPROM
    eeprom_value_t mode = 1;
#else
    eeprom_value_t mode = 0;
#endif

    // CRSF
#ifdef CRSF_ADDRESS
    eeprom_value_t address = CRSF_ADDRESS;
#else
    eeprom_value_t address = 0xc8;
#endif

    eeprom_value_t switchAddress = 0;
#ifdef TEST_EEPROM
    eeprom_value_t switchAddressContiguous = 1;
#else
    eeprom_value_t switchAddressContiguous = 0;
#endif

#ifdef TEST_EEPROM
    eeprom_value_t input_stream = 2;
#else
    eeprom_value_t input_stream = 0; // CRSF, Pulse
#endif

    // Input channels
    std::array<Pair, 2> channels{{{0, 1}, {2, 3}}};

    // escs
    std::array<eeprom_value_t, 2> deadbands{10, 10};

    // servos
    eeprom_value_t offset1  = 0; // 0...360
    eeprom_value_t speed1 = 100;
    eeprom_value_t offset2  = 0;
    eeprom_value_t speed2 = 100;

    // (s)bus out
    // phi1/2, amp1/2 are injected in these channels
    eeprom_value_t inject  = 1;
    eeprom_value_t amp1_ch = 0;
    eeprom_value_t phi1_ch = 1;
    eeprom_value_t amp2_ch = 2;
    eeprom_value_t phi2_ch = 3;

#ifdef TEST_EEPROM
    eeprom_value_t crsf_hd_mode = 9;
#else
    eeprom_value_t crsf_hd_mode = 0;
#endif
    eeprom_value_t crsf_fd_aux_mode = 1;

    eeprom_value_t sport_physicalId_switch = 0;
    eeprom_value_t sport_physicalId_telemetry = 1;

    eeprom_value_t sport_appId_switch = ((uint16_t)RC::Protokoll::SPort::V2::ValueId::DIY) >> 8;
    eeprom_value_t sport_appId_telemetry = ((uint16_t)RC::Protokoll::SPort::V2::ValueId::DIY2) >> 8;

#ifdef TEST_EEPROM
    std::array<eeprom_value_t, 2> out_mode_srv{3, 2};
#else
    std::array<eeprom_value_t, 2> out_mode_srv{2, 2};
#endif

#ifdef TEST_EEPROM
    // std::array<eeprom_value_t, 2> out_mode_esc{2, 0}; // Esc32 Ascii
    std::array<eeprom_value_t, 2> out_mode_esc{4, 4}; // VEsc
#else
    std::array<eeprom_value_t, 2> out_mode_esc{0, 0};
#endif
    std::array<eeprom_value_t, 2> tlm_mode_esc{0, 0};

    std::array<eeprom_value_t, 2> esc_mid{50, 50}; // [0...200]

    struct CompassCalibData {
        int16_t mean = 0;
        int16_t d = 0;
    };
    std::array<CompassCalibData, 3> compass_calib{};

    eeprom_value_t prerun_check = 1;
};
