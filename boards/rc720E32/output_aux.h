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

#include "output.h"
#include "iservo.h"

template<typename Devs>
struct Auxes {
    using devs = Devs;
    using debug = devs::debug;
    using relay = devs::relay_aux;
    // using gps = devs::gps;

    static inline void set(const uint8_t r) {
        IO::outl<debug>("# aux ", r);
        switch(r) {
        case 0: // crsf
            mRelay = nullptr;
            mRelay = std::make_unique<Relay<relay>>();
            break;
        case 1: // gps
            mRelay = nullptr;
            break;
        case 2: // none
            mRelay = nullptr;
            break;
        default:
            break;
        }
    }
#ifdef USE_EXTRA_FORWARDS
    static inline constexpr void forwardPacket(const std::byte type, const std::array<uint8_t, 64>& data, const uint16_t length) {
        if (mRelay) {
            mRelay->forwardPacket(type, data, length);
        }
    }
    static inline void command(const std::array<uint8_t, 64>& data, const uint16_t length){
        if (mRelay) {
            mRelay->command(data, length);
        }
    }
    static inline void ping() {
        if (mRelay) {
            mRelay->ping();
        }
    }
#else
    static inline constexpr void forwardPacket(const auto data, const uint16_t length) {
        if (mRelay) {
            mRelay->forwardPacket(data, length);
        }
    }
#endif
    static inline void update() {
        if (mRelay) {
            mRelay->update();
        }
    }
    static inline void periodic() {
        if (mRelay) {
            mRelay->periodic();
        }
    }
    static inline void ratePeriodic() {
        if (mRelay) {
            mRelay->ratePeriodic();
        }
    }
    private:
    static inline std::unique_ptr<IRelay> mRelay{};
};
