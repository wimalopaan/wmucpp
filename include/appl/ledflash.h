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
#include "util/types.h"

template<typename Led>
class LedFlash {
    LedFlash() = delete;
public:
    typedef typename Led::color_type Color;
    
    static void init() {
        Led::init();
        Led::off();
    }
    static void enable() {
        mFlashTickCount = 0;
    }
    static void disable() {
        mFlashTickCount.setNaN();
    }
    static void tick(const std::percent& brightness) {
        static uint8_t mFlash = 0;
        if (mFlashTickCount) {
            if (mFlashTickCount.toInt() > 0) {
                if ((++mFlash % 2) != 0) {
                    Led::set(mFlashColor * brightness);                
                }
                else {
                    Led::set(mSteadyColor * brightness);
                }
                if (mFlash >= mFlashTickCount.toInt()) {
                    mFlash = 0;
                    mFlashTickCount = 0;
                }
            }
            else {
                update(brightness);
            }
        }
    }
    static void steadyColor(const Color& c) {
        mSteadyColor= c;
    }
    static void update(const std::percent& brightness) {
        Led::set(mSteadyColor * brightness);
    }
    
    static void flash(const Color& c, uint8_t flashCount) {
        assert(flashCount < 128);
        mFlashColor = c;
        if (mFlashTickCount) {
            mFlashTickCount = 2 * flashCount;
        }
    }
    
private:
    inline static uint_NaN<uint8_t> mFlashTickCount{0};
    inline static Color mSteadyColor{0};
    inline static Color mFlashColor{32};
};
