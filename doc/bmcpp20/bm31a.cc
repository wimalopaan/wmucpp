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

#define NDEBUG

#include "mcu/avr8.h"
#include "units/percent.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

using namespace std::literals::quantity;

volatile uint8_t y = 42;
volatile std::percent z = 0_ppc;

static inline
uint8_t scale (uint8_t value, uint8_t min, uint8_t max)
{
    return ((value - min) * 100u) / ((uint16_t) (uint8_t)(max - min));
}

uint8_t scale1 (uint8_t value)
{
    return scale (value, 100, 121);
}

std::percent scale_naiv(uint8_t value, uint8_t min, uint8_t max) {
    if (value < min) {
        return std::percent{0U};
    }
    else if (value > max) {
        return std::percent{100U};
    }
    return std::percent{(uint8_t)(((uint16_t)((value - min) * 100u)) / (uint8_t)(max - min))};
}


int main() {
    y = 42;
    {
        z = scale_naiv(y, uint8_t(100), uint8_t(121));
    }
    {
        z = std::percent(scale1(y));
    }
    while(true) {}
}

template<typename L>
void assertFunction(const PgmStringView&, const PgmStringView&, L) noexcept {
    while(true) {}
}
