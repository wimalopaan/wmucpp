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

template<typename Led>
class TwoStateBlinker {
    TwoStateBlinker() = delete;
public:
    typedef typename Led::color_type Color;
    
    static void init() {
        Led::init();
        Led::off();
    }
    
    static void tick() {
        static bool on = false;
        if (on) {
            Led::set(mOnColor);
            on = false;
        }
        else {
            Led::set(mOffColor);
            on = true;
        }
    }

    static void onColor(const Color& c) {
        mOnColor = c;
    }
    static void offColor(const Color& c) {
        mOffColor = c;
    }

private:
    inline static Color mOffColor{0};
    inline static Color mOnColor{Green{32}};
};
