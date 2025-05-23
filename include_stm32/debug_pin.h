/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

namespace Debug {
    template<typename Pin> struct Scoped;

    template<> struct Scoped<void> {};

    template<typename Pin>
    struct Scoped {
        Scoped() {
            Pin::set();
        }
        ~Scoped() {
            Pin::reset();
        }
    };

    template<typename P> struct Pin;
    template<> struct Pin<void> {
        static inline void set() {
        }
        static inline void reset() {
        }
    };

    template<typename P>
    struct Pin {
        static inline void set() {
            P::set();
        }
        static inline void reset() {
            P::reset();
        }
    };
}
