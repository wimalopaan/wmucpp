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

#include "units/physical.h"

#include "test/simpletest.h"

#undef SIMPLETESTPREFIX
#define SIMPLETESTPREFIX physical

SIMPLETEST("physical01") {
    auto f1 = 1_Hz;
    if (f1.value != 1) return false;
    return true;
};

SIMPLETEST("physical02") {
    auto f1 = 1_MHz;
    if (f1.value != 1) return false;
    return true;
};

SIMPLETEST("physical03") {
    auto f1 = 10_MHz;
    auto f2 = 1_MHz;
    auto q = f1 / f2;
    if (q != 10) return false;
    return true;
};

SIMPLETEST("physical04") {
    auto f1 = 10_MHz;
    auto f2 = f1 / (uint8_t)2;

    if (f2 != 5_MHz) return false;
    return true;
};

SIMPLETEST("physical05") {
    auto f1 = 1_Hz;
    auto d1 = 1_s;

    auto x = d1 * f1;

    if (x != 1) return false;
    return true;
};

SIMPLETEST("physical06") {
    auto f1 = 1_Hz;
    auto d1 = 1_ms;

    auto x = d1 * f1;

    if (x != 0) return false;
    return true;
};

SIMPLETEST("physical07") {
    auto f1 = 1000_Hz;
    auto d1 = 1_ms;

    auto x = d1 * f1;

    if (x != 1) return false;
    return true;
};

SIMPLETEST("physical08") {
    auto f1 = 1000_Hz;
    auto d1 = 1_us;

    auto x = d1 * f1;

    if (x != 0) return false;
    return true;
};

SIMPLETEST("physical09") {
    auto f1 = 1000000_Hz;
    auto d1 = 1_us;

    auto x = d1 * f1;

    if (x != 1) return false;
    return true;
};

SIMPLETEST("physical10") {
    auto f1 = 1000000_Hz;
    auto d1 = 1_ms;

    auto x = d1 * f1;

    if (x != 1000) return false;
    return true;
};

SIMPLETEST("physical11") {
    auto f1 = 1_MHz;
    auto d1 = 1_ms;

    auto x = d1 * f1;

    if (x != 1000) return false;
    return true;
};

SIMPLETEST("physical12") {
    auto f1 = 1_MHz;
    auto d1 = 1_us;

    auto x = d1 * f1;

    if (x != 1) return false;
    return true;
};

SIMPLETEST("physical13") {
    auto f1 = 1000_Hz;

    std::microseconds d = 1 / f1;

    if (d != 1000_us) return false;
    return true;
};
