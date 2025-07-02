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

#include <memory>
#include "port_iface.h"

template<typename Devs>
struct Busses {
    using debug = Devs::debug;
    using relay_bus = Devs::relay_bus;

    static inline void set(const uint8_t a) {
        IO::outl<debug>("# Bus ", a);
        switch(a) {
        case 0: // crsf
        {
            mBus = nullptr;
            mBus = std::make_unique<Bus<relay_bus>>();
            relay_bus::baud(400'000);
        }
            break;
        default:
            mBus = nullptr;
            break;

        }
    }
    static inline void periodic() {
        if (mBus) {
            mBus->periodic();
        }
    }
    static inline void ratePeriodic() {
        if (mBus) {
            mBus->ratePeriodic();
        }
    }
    static inline constexpr void forwardPacket(const auto data, const uint16_t length) {
        if (mBus) {
            mBus->forwardPacket(data, length);
        }
    }
    private:
    static inline std::unique_ptr<IBus> mBus{};
};
