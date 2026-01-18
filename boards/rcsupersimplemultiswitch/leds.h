/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <etl/meta.h>

template<typename List> struct Leds;
template<typename... L>
struct Leds<Meta::List<L...>> {
    using list = Meta::List<L...>;
    static inline void init() {
        (L::init(), ...);
        allOff();
    }
    static inline void set(const uint8_t led, const bool on) {
        Meta::visitAt<list>(led, [&]<typename LL>(Meta::Wrapper<LL>){
                                if (on) {
                                    LL::activate();
                                }
                                else {
                                    LL::inactivate();
                                }
                               });
    }
    static inline void allOff() {
        Meta::visit<list>([&]<typename LL>(Meta::Wrapper<LL>){
                                   LL::inactivate();
                               });

    }
};

