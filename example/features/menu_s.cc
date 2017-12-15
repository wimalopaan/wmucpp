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

#include <tuple>

struct MenuItem {
};

template<typename... CC>
struct Menu1 {
    Menu1(CC... cc) : mChildren{cc...}{
    }    
    std::tuple<CC...> mChildren;    
};
struct Menu2 {
};
struct Menu3 {
};

Menu2 m2;
Menu3 m3;

Menu1 m1(&m2, &m3);


int main() {
}