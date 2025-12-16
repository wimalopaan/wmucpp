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

#include <cstdint>
#include <array>

struct EEProm {
    constexpr EEProm() {
        etl::copy("RelayTX1", outputParams[0].tx_name);
        etl::copy("RelayTX2", outputParams[1].tx_name);
        etl::copy("RelayTX3", outputParams[2].tx_name);
        etl::copy("RelayTX4", outputParams[3].tx_name);
        etl::copy("RelayTX5", outputParams[4].tx_name);
        etl::copy("RelayTX6", outputParams[5].tx_name);
    }

    uint8_t address = 0xc0;
    uint8_t commandBroadcastAddress = 0xc8; // command packages with this dest-address are routed to all interfaces

    struct OutputParam {
        uint8_t forwardLinkStats  = 1;
        uint8_t forwardRCChannels = 1;
        uint8_t forwardBCast      = 1;
        uint8_t asRelay           = 0;
        uint8_t baudrate          = 1;
        std::array<char, 16>       tx_name{};
        uint8_t forward_link_stats_as_tunnel_package = 1;
        uint8_t failsafe_mode       = 1;
        uint8_t telemetry_forward   = 1;
        uint8_t telemetry_tunnel    = 1;
        uint8_t telemetry_rate      = 2;
        uint8_t rewrite_name        = 1;
    };
    std::array<OutputParam, 6> outputParams = {};

};
