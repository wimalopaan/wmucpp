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

#define NDEBUG // don't change (enables assertions)

#include "settings.h"
#include "devices.h"
#include "gfsm.h"

using namespace External::Units::literals;

template<typename Protocoll>
struct DevsConfig {
    static inline constexpr auto fRtc = 2000_Hz; // timing 500µs resolution
    using protocoll = Protocoll;
};

int main() {
#if defined(INPUT_CRSF)
    using devsConfig = DevsConfig<Protocoll::Crsf>;
    using devices = Devices<SSMSW01, devsConfig>;
    using gfsm = Gfsm::WithTelemetry<devices>;
#elif defined(INPUT_IBUS)
    using devsConfig = DevsConfig<Protocoll::IBus>;
    using devices = Devices<SSMSW01, devsConfig>;
    using gfsm = Gfsm::NoTelemetry<devices>;
#elif defined(INPUT_SBUS)
    using devsConfig = DevsConfig<Protocoll::SBus>;
    using devices = Devices<SSMSW01, devsConfig>;
    using gfsm = Gfsm::NoTelemetry<devices>;
#elif defined(INPUT_SPORT)
    using devsConfig = DevsConfig<Protocoll::SPort>;
    using devices = Devices<SSMSW01, devsConfig>;
    using gfsm = Gfsm::NoTelemetry<devices>;
#else
#error "wrong input selection"
#endif

    gfsm::init();
    while(true) {
        gfsm::periodic();
        devices::systemTimer::periodic([] static {
                                           gfsm::ratePeriodic();
                                       });
    }
}
