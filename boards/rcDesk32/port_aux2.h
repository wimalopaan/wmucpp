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
struct Auxes2 {
    using debug = Devs::debug;
    using sbus2 = Devs::sbus2;
    using hwext2 = Devs::hwext2;

    static inline void set(const uint8_t a) {
        IO::outl<debug>("# Auxes2 ", a);
        switch(a) {
        case 0: // sbus
        {
            mAux = nullptr;
            mAux = std::make_unique<Aux<sbus2>>();
        }
            break;
        case 1: // inv sbus
        {
            mAux = nullptr;
            mAux = std::make_unique<Aux<sbus2>>();
            sbus2::invert(false);
        }
            break;
        case 2: // hwext
        {
            mAux = nullptr;
            mAux = std::make_unique<Aux<hwext2>>();
        }
            break;
        default:
            mAux = nullptr;
            break;

        }
    }
    static inline void periodic() {
        if (mAux) {
            mAux->periodic();
        }
    }
    static inline void ratePeriodic() {
        if (mAux) {
            mAux->ratePeriodic();
        }
    }
    private:
    static inline std::unique_ptr<IAux> mAux{};

};
