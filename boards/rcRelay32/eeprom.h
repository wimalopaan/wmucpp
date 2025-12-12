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
    uint32_t magic = EEPROM_MAGIC;

    constexpr EEProm() {
        etl::copy("RelayTX", txname);
    }
    std::array<char, 8> txname{};

	uint8_t rewrite_name = 0;
	uint8_t forward_link_stats_as_tunnel_package = 0;

    uint8_t address = 0xcd;
    uint8_t commandBroadcastAddress = 0xc8; // command packages with this dest-address are routed to all interfaces

	uint8_t tx_rewrite_address = 0xce;
	uint8_t rx_rewrite_address = 0xcf;

    uint8_t half_duplex = 0;

    uint8_t telemetry_tunnel = 0;
    uint8_t telemetry_forward = 0;
    uint8_t telemetry_rate = 2;
};
