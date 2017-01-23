/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "units/percent.h"

#include "test/simpletest.h"

#undef SIMPLETESTPREFIX
#define SIMPLETESTPREFIX percent

SIMPLETEST("percent01") {
    using namespace std::literals::quantity;
    auto x = 10_ppc;
    if (x != 10_ppc) return false;
    return true;
};

SIMPLETEST("percent02") {
    using namespace std::literals::quantity;
    auto x = std::scale(50, 0, 100);
    if (x != 50_ppc) return false;
    return true;
};

SIMPLETEST("percent01") {
    using namespace std::literals::quantity;
    auto x = 10_ppc;
    if (x != 10_ppc) return false;
    return true;
};

SIMPLETEST("percent03") {
    using namespace std::literals::quantity;
    auto x = std::scale(-1, 0, 100);
    if (x != 0_ppc) return false;
    return true;
};

SIMPLETEST("percent04") {
    using namespace std::literals::quantity;
    auto x = std::scale(200, 0, 100);
    if (x != 100_ppc) return false;
    return true;
};

SIMPLETEST("percent05") {
    using namespace std::literals::quantity;
    auto x = std::expand(10_ppc, 0, 100);
    if (x != 10) return false;
    return true;
};

SIMPLETEST("percent06") {
    using namespace std::literals::quantity;
    uint8_t a = 0;
    uint8_t b = 100;
    uint8_t x = std::expand(10_ppc, a, b);
    if (x != 10) return false;
    return true;
};

SIMPLETEST("percent07") {
    using namespace std::literals::quantity;
    uint8_t a = 0;
    uint8_t b = 250;
    uint8_t x = std::expand(10_ppc, a, b);
    if (x != 25) return false;
    return true;
};

SIMPLETEST("percent08") {
    using namespace std::literals::quantity;
    uint16_t a = 0;
    uint16_t b = 250;
    uint16_t x = std::expand(10_ppc, a, b);
    if (x != 25) return false;
    return true;
};

SIMPLETEST("percent09") {
    using namespace std::literals::quantity;
    uint16_t a = 0;
    uint16_t b = 65000;
    uint16_t x = std::expand(10_ppc, a, b);
    if (x != 6502) return false;
    return true;
};
