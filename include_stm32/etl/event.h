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

namespace etl {
    template<typename T>
    struct Event {
        void operator=(const T e) volatile {
            mEvent = e;
        }
        void operator=(const T e) {
            mEvent = e;
        }
        void on(const T which, const auto f) {
            if (is(which)) {
                f();
            }
        }
        bool is(const T which) {
            if (mEvent == which) {
                mEvent = T::None;
                return true;
            }
            return false;
        }
        bool is(const T which) volatile {
            if (mEvent == which) {
                mEvent = T::None;
                return true;
            }
            return false;
        }
        private:
        T mEvent = T::None;
    };
}



