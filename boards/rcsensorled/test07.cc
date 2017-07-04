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

//#define MEM
//#define NDEBUG

#include "local.h"
#include "rcsensorled01.h"
#include "console.h"
#include "util/meta.h"

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

using distributor = Distributor<terminalDevice, adcController, ds18b20Sync>;

std::array<OneWire::ow_rom_t, 5> dsIds;

int main() {
    distributor::init();

    std::outl<terminal>("Test07"_pgm);

    oneWireMaster::findDevices(dsIds, ds18b20::family);
    for(const auto& id : dsIds) {
        std::outl<terminal>(id);
    }
    
    while(true) {
        ds18b20Sync::convert();
        Util::delay(1000_ms);
        for(const auto& id : dsIds) {
            if (id) {
                ds18b20Sync::ds18b20_rsp_t sp;
                ds18b20Sync::readScratchpad(id, sp);
                auto t = ds18b20Sync::temperature(sp);
                std::outl<terminal>(t);
            }
        }
    }
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    while(true) {}
}
#endif
