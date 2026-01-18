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

#include "mcu/eeprom.h"
#include "meta.h"
#include "eeprom.h"
#include "rc/rc_2.h"
#include "rc/crsf_2.h"

template<typename Src, typename Buffer>
struct SwitchCallback {
    using src = Src;
    using buffer = Buffer;

    static inline void update() {
        uint64_t s{};
        for(uint8_t i = 0; i < 8; ++i) {
            if (src::switches()[0].values[i] == 1) {
                s |= (uint64_t{0b01} << (4 * i));
            }
            else if (src::switches()[0].values[i] == 2) {
                s |= (uint64_t{0b01} << (4 * i + 2));
            }
        }
        for(uint8_t i = 0; i < 8; ++i) {
            if (src::switches()[1].values[i] == 1) {
                s |= (uint64_t{0b01} << (4 * i + 32));
            }
            else if (src::switches()[1].values[i] == 2) {
                s |= (uint64_t{0b01} << (4 * i + 2 + 32));
            }
        }
        buffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::Command, [&](auto& d){
            d.push_back((uint8_t)0xc8);
            d.push_back(RC::Protokoll::Crsf::V4::Address::Handset);
            d.push_back(RC::Protokoll::Crsf::V4::CommandType::Switch);
            d.push_back(RC::Protokoll::Crsf::V4::SwitchCommand::Set4M);
            d.push_back((uint8_t)4);
            d.push_back((uint8_t)240);
            uint16_t sw = s & 0xffff;
            d.push_back(sw);
            d.push_back((uint8_t)241);
            sw = (s >> 16) & 0xffff;
            d.push_back(sw);
            d.push_back((uint8_t)242);
            sw = (s >> 32) & 0xffff;
            d.push_back(sw);
            d.push_back((uint8_t)243);
            sw = (s >> 48) & 0xffff;
            d.push_back(sw);
        });
    }
};
