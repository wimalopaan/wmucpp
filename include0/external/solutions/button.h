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

#pragma once

#include <cstdint>
#include <cstddef>

#include <std/chrono>
#include <etl/rational.h>
#include <etl/concepts.h>
#include <etl/algorithm.h>

#include "mcu/common/concepts.h"

namespace External {
    template<typename Pin, uint8_t PressCounts = 10>
    struct Button {
        static inline void init() {
            Pin::init();
        }
        template<etl::Concepts::Callable F>
        static inline void periodic(const F& f) {
            if (Pin::isActive()) {
                if (++mCounter == PressCounts) {
                    f();
                }
            }
            else {
                mCounter = 0;
            }
        }
    private:
        static inline etl::uint_ranged<uint8_t, PressCounts + 1> mCounter = 0;
    };
}
