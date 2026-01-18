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

#include <memory>
#include "port_iface.h"

template<typename Devs>
struct Smes2 {
    using debug = Devs::debug;
    using sm2 = Devs::sm2;

    static inline void set(const uint8_t a) {
        IO::outl<debug>("# Smes2 ", a);
        switch(a) {
        case 0: // SM
        {
            mSm = nullptr;
            mSm = std::make_unique<Sm<sm2>>();
        }
            break;
        default:
            mSm = nullptr;
            break;

        }
    }
    static inline void periodic() {
        if (mSm) {
            mSm->periodic();
        }
    }
    static inline void ratePeriodic() {
        if (mSm) {
            mSm->ratePeriodic();
        }
    }
    private:
    static inline std::unique_ptr<ISm> mSm{};

};
