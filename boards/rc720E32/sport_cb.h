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

#include "mcu/eeprom.h"
#include "meta.h"
#include "eeprom.h"
#include "uuid.h"
#include "rc/rc_2.h"

template<typename Config>
struct SPortCallback {
    using debug = Config::debug;
    using timer = Config::timer;
    using tp = Config::tp;
    using storage = Config::storage;

    using mpx1 = Config::mpx1;
    using sumdv3 = Config::sumdv3;

    static inline void command(const volatile uint8_t* const data) {
        const uint8_t command = data[0];
        const uint16_t appId = data[1] + (data[2] << 8);
        const uint32_t value = data[3] + (data[4] << 8) + (data[5] << 16) + (data[6] << 24);

        if (appId == mSW_0_31) {
            IO::outl<debug>("# SPort cb: ", command, ", ", appId, ", ", value);
        }
        else if (appId == mSW_32_63) {
            IO::outl<debug>("# SPort cb: ", command, ", ", appId, ", ", value);
        }
    }
    private:
    static inline uint16_t mSW_0_31 = 0xac00;
    static inline uint16_t mSW_32_63 = 0xac01;

};
