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

#include <cstdint>

template<typename R>
struct RessourceCount {
    static inline void acquire(const auto f) {
        if (mCounter == 0) {
            f();
        }
        ++mCounter;
    }
    static inline void release(const auto f) {
        if (mCounter > 0) {
            if (--mCounter == 0) {
                f();
            }
        }
    }
    private:
    static inline uint8_t mCounter = 0;
};
