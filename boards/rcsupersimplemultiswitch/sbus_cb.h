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
struct SBusCommandCallback {
    using debug = Config::debug;
    using ledGroups = Config::ledGroups;
    using leds = Meta::nth_element<0, ledGroups>;

    static inline void decode(const std::array<uint8_t, 23>& data) {
        const uint16_t ch16 = (uint16_t) ((data[20]>>5 | data[21]<<3) & 0x07FF);
#if defined(USE_ELRS)
        static constexpr uint16_t value0 = 172;
        const uint16_t n = (ch16 >= value0) ? (ch16 - value0) : 0;
        const uint8_t  v = (n >> 4);
#elif defined(USE_AFHDS2A)
        static constexpr uint16_t value0 = 220;
        const uint16_t n = (ch16 >= value0) ? (ch16 - value0) : 0;
        const uint16_t n2 = n + (n >> 6);
        const uint8_t  v = (n2 >> 4);
#elif defined(USE_ACCST)
        static constexpr uint16_t value0 = 172;
        const uint16_t n = (ch16 >= value0) ? (ch16 - value0 + 1) : 0;
        const uint8_t  v = (n >> 4);
#else
#error "wrong protocol"
#endif
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
    static inline uint8_t mAddress = DEFAULT_ADDRESS;
};

