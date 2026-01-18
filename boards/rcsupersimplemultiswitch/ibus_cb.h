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
struct IBusCommandCallback {
    using debug = Config::debug;
    using ledGroups = Config::ledGroups;
    using leds = Meta::nth_element<0, ledGroups>;

    static inline void decode(const std::array<std::byte, 28>& data) {
        ++mCommandPackagesCounter;
        static constexpr uint8_t chi = 15;
        const std::byte h1 = data[6 * (chi - 14) + 1] & 0xf0_B;
        const std::byte h2 = data[6 * (chi - 14) + 3] & 0xf0_B;
        const std::byte h3 = data[6 * (chi - 14) + 5] & 0xf0_B;
        const uint16_t ch16 = (uint8_t(h1) >> 4) + uint8_t(h2) + (uint16_t(h3) << 4);

        static constexpr uint16_t value0 = 988; // 2011
        const uint16_t n = (ch16 >= value0) ? (ch16 - value0 + 1) : 0;
        const uint8_t  v = (n >> 4);

        const uint8_t address = (v >> 4) & 0b11;
        const uint8_t sw      = (v >> 1) & 0b111;
        const uint8_t state   = v & 0b1;

        if (v != mLastCommand) {
            mLastCommand = v;
            etl::outl<debug>("ch16: "_pgm, ch16, ", adr: "_pgm, address, " sw: "_pgm, sw, ", state: "_pgm, state);
            if (address == mAddress) {
                const bool on = (state == 1);
                leds::set(sw, on);
            }
        }
    }
    static inline void address(const uint8_t adr) {
        mAddress = adr;
    }
    static inline uint8_t address() {
        return mAddress;
    }
    private:
    static inline uint8_t mLastCommand = 0;
    static inline uint16_t mCommandPackagesCounter = 0;
    static inline uint8_t mAddress = DEFAULT_ADDRESS;
};

