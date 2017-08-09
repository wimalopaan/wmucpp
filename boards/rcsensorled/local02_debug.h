/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "std/literals.h"

#define HAS_CONFIG

using namespace std::literals::chrono;
using namespace std::literals::physical;

struct Config final
{
    Config() = delete; // one should use a similar copy in own project

    inline static constexpr std::megahertz fMcuMhz {F_CPU / 1000000};
    inline static constexpr std::hertz fMcu{F_CPU};

    static_assert(fMcuMhz.value <= 20, "F_CPU too high");
    static_assert(fMcuMhz.value >=  1, "F_CPU too low");

    struct Timer {
        inline static constexpr uint8_t NumberOfTimers = 8;
        inline static constexpr std::hertz frequency = 500_Hz;
        inline static constexpr std::milliseconds resolution = std::duration_cast<std::milliseconds>(1 / frequency);
    };
    struct EventManager {
        inline static constexpr uint8_t EventQueueLength = 32;
    };
    struct Usart {
        inline static constexpr uint16_t SendQueueLength = 256;
        inline static constexpr uint8_t RecvQueueLength = 0;
        inline static constexpr bool  useEvents = true;
    };
    struct SoftSpiMaster {
        inline static constexpr std::microseconds pulseDelay = 1_us;
    };
    struct Button {
        inline static constexpr uint8_t buttonTicksForPressed = 100_ms * Timer::frequency;
    };
    inline static constexpr std::microseconds zeroMicroSeconds{0};
    inline static constexpr bool ensureTerminalOutput = false;
    inline static constexpr bool disableCout = false;
};

