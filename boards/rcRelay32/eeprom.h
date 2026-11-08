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
#include <algorithm>

#include "rc/rc_2.h"

#define EEPROM_MAGIC 42

struct EEProm {
    uint32_t magic = EEPROM_MAGIC;

    constexpr EEProm() {
        etl::copy("RelayTX", txname);
        for(auto& m : telemetryMode) {
            m = 1;
        }
    }
    std::array<char, 8> txname{};

	uint8_t rewrite_name = 0;
    uint8_t link_stat_mode = 2; // transform

    uint8_t failsafe_mode = 0;

    uint8_t address = 0xcd;
    uint8_t commandBroadcastAddress = 0xc8; // command packages with this dest-address are routed to all interfaces

	uint8_t tx_rewrite_address = 0xce;
	uint8_t rx_rewrite_address = 0xcf;

    uint8_t half_duplex = 0;
    uint8_t irda = 0;
    
    uint8_t baudrate       = 3;
    uint8_t telemetry_rate = 2;
    
    uint8_t tempOffTelemetry = 30; // seconds for temporary telemetry off after parameter request

    std::array<uint8_t, (uint8_t)RC::Protokoll::Crsf::V4::Type::Ping> telemetryMode{};

    uint8_t ardupilot_mode = 1; // ArduPilot passthrough (0x80): 0=Off, 1=Forward, 2=Tunnel

    constexpr bool valid() const {
        return (magic == EEPROM_MAGIC) &&
            (std::find(txname.begin(), txname.end(), '\0') != txname.end()) &&
            (rewrite_name <= 1) && (link_stat_mode <= 3) && (failsafe_mode <= 1) &&
            (address >= 192) && (address <= 207) &&
            (tx_rewrite_address >= 192) && (tx_rewrite_address <= 207) &&
            (rx_rewrite_address >= 192) && (rx_rewrite_address <= 207) &&
            (half_duplex <= 1) && (irda <= 1) &&
            (baudrate < RC::Protokoll::Crsf::V4::baudratesHandset.size()) &&
            (telemetry_rate >= 1) && (telemetry_rate <= 10) &&
            (tempOffTelemetry >= 1) && (tempOffTelemetry <= 60) &&
            (ardupilot_mode <= 2) &&
            std::all_of(telemetryMode.begin(), telemetryMode.end(), [](const uint8_t mode) {
                return mode <= 2;
            });
    }
};
