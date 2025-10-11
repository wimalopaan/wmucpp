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

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>

namespace Project {
    template<typename Config = void>
    struct Clock {
        using ccp = AVR::Cpu::Ccp<>;
        using clock = AVR::Clock<>;
        static inline void init() {
            ccp::unlock([]{
                clock::template init<Project::Config::fMcuMhz>();
            });
        }
    };
}

