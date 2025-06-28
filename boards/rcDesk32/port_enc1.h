/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
struct Enc1s {
    using debug = Devs::debug;
    using enc1 = Devs::enc1;

    static inline void set(const uint8_t a) {
        IO::outl<debug>("# Enc1 ", a);
        switch(a) {
        case 0: // Enc1
        {
            mEnc = nullptr;
            mEnc = std::make_unique<Enc<enc1>>();
        }
            break;
        default:
            mEnc = nullptr;
            break;

        }
    }
    static inline void periodic() {
        if (mEnc) {
            mEnc->periodic();
        }
    }
    static inline void ratePeriodic() {
        if (mEnc) {
            mEnc->ratePeriodic();
        }
    }
    private:
    static inline std::unique_ptr<IEnc> mEnc{};
};
