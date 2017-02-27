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

#pragma once

#include <stdint.h>

template<typename Led>
class LedFlash {
    LedFlash() = delete;
public:
    typedef typename Led::color_type Color;
    
    static void init() {
        Led::init();
        Led::off();
    }
    
    static void tick() {
        static uint8_t mFlash = 0;
        if (mFlashTickCount > 0) {
            if ((++mFlash % 2) != 0) {
                Led::set(mFlashColor);                
            }
            else {
                Led::set(mSteakColor);
            }
            if (mFlash >= mFlashTickCount) {
                mFlash = mFlashTickCount = 0;
            }
        }
    }

    static void steadyColor(const Color& c) {
        mSteakColor= c;
        Led::set(mSteakColor);
    }
    static void flash(const Color& c, uint8_t flashCount) {
        assert(flashCount < 128);
        mFlashColor = c;
        mFlashTickCount = 2 * flashCount;
    }

private:
    static uint8_t mFlashTickCount;
    static Color mSteakColor;
    static Color mFlashColor;
};

template<typename Led>
uint8_t LedFlash<Led>::mFlashTickCount{0};

template<typename Led>
typename LedFlash<Led>::Color LedFlash<Led>::mSteakColor{0};
template<typename Led>
typename LedFlash<Led>::Color LedFlash<Led>::mFlashColor{32};
