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

//#include "std/array.h"
//#include <iostream>

template<int... N>
struct expand;

template<int... N>
struct expand<0, N...>
{
//    constexpr static std::array<int, sizeof...(N) + 1> values = {{ 0, N... }};
    constexpr static int values[sizeof...(N) + 1] = { 0, N... };
};

template<int L, int... N> 
struct expand<L, N...> : expand<L-1, L, N...> 
{
};

//template<int... N>
//constexpr std::array<int, sizeof...(N) + 1> expand<0, N...>::values;

int main()
{
    int x = expand<3>::values[0];
    
    
}