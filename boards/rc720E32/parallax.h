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

#include <numbers>
#include <chrono>

#include "etl/event.h"

#include "ressources.h"
#include "pid.h"

using namespace std::literals::chrono_literals;

template<typename Config>
struct Parallax {
    using storage = Config::storage;
    using clock = Config::clock;
    using timer = Config::timer;
    using out = Config::out;
    using in = Config::in;
    using input = Config::polar;
    using debug = Config::debug;

    static inline void init() {
        IO::outl<debug>("# Parallay init");
        // in::init();
        out::init();
    }    
    static inline void reset() {
        IO::outl<debug>("# Parallay reset");
    }    
};
