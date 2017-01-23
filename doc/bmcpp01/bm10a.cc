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

#include <stdint.h>
#include "std/types.h"
#include "std/optional.h"

volatile uint7_t global{3};
volatile uint7_t global2{0};
volatile bool b = false;

////[opt
std::optional<uint7_t> foo1() {
    if (b) {
        return global;
    }
    return {};
}
//]
//[main
int main()
{
    if (auto r = foo1()) {
        global2 = *r;
    }

    while(true);
}
//]
