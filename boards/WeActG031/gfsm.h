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

#include "rc/rc_2.h"
#include "etl/event.h"
#include <tick.h>

using namespace std::literals::chrono_literals;

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using systemTimer = devs::systemTimer;
    using led = devs::ledBlinker;

    static inline void init() {
        devs::init();
        if constexpr(!std::is_same_v<led, void>) {
            led::event(led::Event::Fast);
        }
    }
    static inline void periodic() {
		devs::periodic();
    }
    static inline void ratePeriodic() {
		devs::ratePeriodic();
    }
};
