/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <stdint.h>

#include "std/literals.h"

#define HAS_CONFIG

using namespace std::literals::chrono;
using namespace std::literals::physical;

struct Config final
{
    static constexpr std::megahertz fMcuMhz {F_CPU / 1000000};
    static constexpr std::hertz fMcu{F_CPU};

    static_assert(fMcuMhz.value <= 20, "F_CPU too high");
    static_assert(fMcuMhz.value >=  1, "F_CPU too low");

    struct Timer {
        static constexpr uint8_t NumberOfTimers = 8;
        static constexpr std::hertz frequency = 100_Hz;
        static constexpr std::milliseconds resolution = std::duration_cast<std::milliseconds>(1 / frequency);
    };
    struct EventManager {
        static constexpr uint8_t EventQueueLength = 32;
    };
    struct Usart {
        typedef uint8_t SizeType;
        static constexpr SizeType SendQueueLength = 64;
        static constexpr SizeType RecvQueueLength = 0;
    };
    struct SoftSpiMaster {
        static constexpr std::microseconds pulseDelay = 1_us;
    };
    struct Button {
        static constexpr uint8_t buttonTicksForPressed = 100_ms * Timer::frequency;
    };

    static constexpr bool ensureTerminalOutput = true;
    static constexpr bool disableCout = false;
};

constexpr std::hertz Config::fMcu;
constexpr std::megahertz Config::fMcuMhz;
constexpr uint8_t Config::Timer::NumberOfTimers;
constexpr std::hertz Config::Timer::frequency;
constexpr std::milliseconds Config::Timer::resolution;
constexpr uint8_t Config::EventManager::EventQueueLength;
constexpr Config::Usart::SizeType Config::Usart::SendQueueLength;
constexpr Config::Usart::SizeType Config::Usart::RecvQueueLength;
constexpr bool Config::ensureTerminalOutput;
constexpr bool Config::disableCout;
constexpr std::microseconds Config::SoftSpiMaster::pulseDelay;
