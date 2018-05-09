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

//#define MEM
#define NDEBUG

#include "rcsensorled01.h"
#include "console.h"
#include "util/meta.h"

class MenuItem {
public:
    virtual void update() = 0;
//    virtual ~MenuItem() = default;
private:
};

class Info : public MenuItem {
public:
    explicit Info(PgmStringView text) : mText(text) {}
    virtual void update() override {
    }
private:
    const PgmStringView mText;
    
};

class Menu : public MenuItem
{
public:
    explicit Menu(PgmStringView title) 
        : mTitle(title)
    {}
    virtual void update() override {
        
    }
private:
    const PgmStringView mTitle;
//    std::array<MenuItem*, 7> mItems;
};


Menu startMenu("Title"_pgm);

Info i("bla"_pgm);
Info i2("bla"_pgm);

namespace {
    constexpr bool useTerminal = true;
}

using terminalDevice = std::conditional<useTerminal, SSpi0, void>::type;
using terminal = std::basic_ostream<terminalDevice>;

int main() {
    terminalDevice::init();
    
//    std::outl<terminal>("Test99"_pgm);
    
    while(true) {
    }
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    while(true) {}
}
#endif
