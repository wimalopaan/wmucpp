/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "container/pgmstring.h"

#define PGMSUFFIX pgm
#define PASTER(x,y) x ## _ ## y
#define EVALUATOR(x,y)  PASTER(x,y)
#define PGMSTRING(string) EVALUATOR(string, PGMSUFFIX)

extern void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept __attribute__ ((__noreturn__));

#ifndef NDEBUG
# define assert(expr) ((expr) ? (void) (0) : assertFunction(PgmStringView{PGMSTRING(#expr).data}, PgmStringView{PGMSTRING(__FILE__).data}, __LINE__))
#else
# define assert(x)
#endif

//// todo: Umstellen auf pgm Strings
//extern void assertFunction(const char* expr, const char* function, const char* file, unsigned int line) noexcept __attribute__ ((__noreturn__));
//#ifndef NDEBUG
//# define assert(expr) ((expr) ? (void) (0) : assertFunction (#expr, __PRETTY_FUNCTION__, __FILE__, __LINE__))
//#else
//# define assert(x)
//#endif
