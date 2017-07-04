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

// preserve eeprom
// sudo avrdude -p atmega328pb -C+/home/lmeier/Projekte/wmucpp/avrdude.conf -P usb -c avrisp2 -U lfuse:w:0xe0:m -U hfuse:w:0xd1:m -U efuse:w:0xff:m

//#define MEM
#define NDEBUG

#include "rcsensorled01.h"
#include "hal/eeprom.h"
#include "util/meta.h"
#include "container/stringbuffer.h"
#include "console.h"

namespace {
    constexpr bool useTerminal = true;
}

using terminalDevice = std::conditional<useTerminal, SSpi0, void>::type;
using terminal = std::basic_ostream<terminalDevice>;

template<typename... T>
struct Distributor {
    using Items = Meta::filter<Meta::nonVoid, Meta::List<T...>>;
    template<typename U> struct NonVoidDistributor;
    template<template<typename...> typename L, typename... U>
    struct NonVoidDistributor<L<U...>> {
        inline static void init() {
            (U::init(), ...);
        }
    };
    inline static void init() {
        NonVoidDistributor<Items>::init();
    }
};

class EEPromData : public EEPromBase<EEPromData>{
public:
    StringBuffer<10>& text() {
        return mText;
    }
    
private:
    StringBuffer<10> mText;
};

using eeprom = EEProm<EEPromData>;

using distributor = Distributor<terminalDevice, eeprom>;

int main() {
    distributor::init();
    std::outl<terminal>("Test11"_pgm);
    
    eeprom::data().text().insertAtFill(0, "Bla Bla"_pgm);
    eeprom::data().change();
    eeprom::data().expire();
    
    std::outl<terminal>("Text: "_pgm, eeprom::data().text());   
    std::outl<terminal>("save ..."_pgm);

    while(eeprom::saveIfNeeded()) {
        std::outl<terminal>("."_pgm);
    }
  
    std::outl<terminal>("...saved "_pgm);
    while(true) {}
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    while(true) {}
}
#endif
