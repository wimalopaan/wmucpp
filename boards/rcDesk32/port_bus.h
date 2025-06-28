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
    using bus_crsf = Devs::bus_crsf;

    static inline void set(const uint8_t a) {
        IO::outl<debug>("# Bus ", a);
        switch(a) {
        case 0: // crsf
        {
            mBus = nullptr;
            mBus = std::make_unique<Bus<bus_crsf>>();
            bus_crsf::baud(400'000);
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
    private:
    static inline std::unique_ptr<IBus> mBus{};
};
