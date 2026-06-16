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

template<typename Config>
struct SumDV3CommandCallback {
    using debug = Config::debug;
    using ledGroups = Config::ledGroups;
    using leds = Meta::nth_element<0, ledGroups>;

    static inline void decode(const std::array<std::byte, 36>& data) {
        const std::byte fcode = data[32];
        if (fcode == 0x05_B) {
            ++mCommandPackagesCounter;
            const uint8_t offset = 24 + mGroup; // 24: start of switches
            const uint16_t state = (uint16_t)data[offset] | ((uint16_t)data[offset + 1] << 8);
            const uint16_t changed = (state ^ mLastState);
            for(uint8_t i = 0; i < 16; ++i) {
                const uint16_t mask = (0x01 << i);
                if (changed & mask) {
                    leds::set(i, (state & mask));
                }
            }
            mLastState = state;
        }
    }
    static inline void address(const uint8_t adr) {
        if (adr < 7) {
            mGroup = adr;
        }
    }
    static inline uint8_t address() {
        return mGroup;
    }
    private:
    static inline uint16_t mLastState = 0;
    static inline uint16_t mCommandPackagesCounter = 0;
    static inline uint8_t mGroup = DEFAULT_ADDRESS;
};

