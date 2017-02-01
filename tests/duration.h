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

#include "units/duration.h"
#include "units/physical.h"

#include "test/simpletest.h"

#undef SIMPLETESTPREFIX
#define SIMPLETESTPREFIX duration

SIMPLETEST("duration01") {
    auto d = 1_ms;
    if (d.value != 1) return false;
    return true;
};

SIMPLETEST("duration02") {
    auto d = 1_us;
    if (d.value != 1) return false;
    return true;
};

SIMPLETEST("duration03") {
    auto d = 1_s;
    if (d.value != 1) return false;
    return true;
};

SIMPLETEST("duration04") {
    auto d = 1_s;
    std::milliseconds d2 = d;
    if (d2.value != 1000) return false;
    return true;
};

SIMPLETEST("duration05") {
    auto d1 = 20_ms;
    auto d2 = 5_ms;
    auto q = d1 / d2;
    if (q != 4) return false;
    return true;
};
