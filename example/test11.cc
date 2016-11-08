/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

#include <util/delay.h>
#include "pgmstring.h"

int main()
{
    auto fs1 = "xydd"_pgm;
    {
        const char* ptr = fs1.str();
        while (char c = pgm_read_byte(ptr++)) {
            _delay_us(1.0);
        };
    }
    auto fs2 = "xydd"_pgm;
    {
        const char* ptr = fs2.str();
        while (char c = pgm_read_byte(ptr++)) {
            _delay_us(1.0);
        };
    }
}
