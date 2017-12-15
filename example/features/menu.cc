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

#include "ui/menu.h"

static constexpr uint8_t MenuStringLength = 20;
using BufferString = StringBuffer<MenuStringLength + 1>;
using MenuItem = UI::MenuItem<BufferString, uint8_t>;

template<uint8_t MenuLength>
class Menu : public MenuItem {
public:
private:
    Menu* mParent = nullptr;
    const PgmStringView mTitle;
    const std::array<MenuItem*, MenuLength> mItems;
    uint_ranged_NaN<uint8_t, 0, MenuLength - 1> mSelectedLine;
};


AMenu menu;

volatile uint8_t key;

int main() {
    MenuItem* actual = &menu;
    while (true) {
        actual = actual->processKey(key);                
    }    
}