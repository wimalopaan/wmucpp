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

#include "util/types.h"
#include "container/fixedvector.h"
#include "external/ws2812.h"

template<typename Led, const typename Led::color_type& SteadyColor, uint8_t MaximumColors = 2>
class Blinker {
    Blinker() = delete;
public:
    typedef typename Led::color_type Color;
    
    static void init() {
        Led::init();
        Led::off();
    }
    static void blink(const Color& color, uint8_t blinks) {
        colors.clear();
        colors.push_back(Color{0});
        colors.push_back(color);
        count = blinks;
        actualColor = 0;
        actualCount = 0;
    }
    static void blink(const Color& color1, const Color& color2, uint8_t blinks) {
        colors.clear();
        colors.push_back(color1);
        colors.push_back(color2);
        count = blinks;
        actualColor = 0;
        actualCount = 0;
    }
    template<uint8_t N>
    static void blink(const std::array<Color, N>& bcolors, uint8_t blinks) {
        static_assert(N <= MaximumColors, "too many colors");
        colors.clear();
        for( const auto& c: bcolors) {
            colors.push_back(c);
        }
        count = blinks;
        actualColor = 0;
        actualCount = 0;
    }
        
    static void tick() {
        if (count) {
            if (actualCount < *count) {
                assert(actualColor < colors.size());
                Led::set(colors[actualColor]);
                actualColor = (actualColor + 1) % colors.size();
                if (actualColor == 0) {
                    ++actualCount;
                }
            }
            else {
                count.setNaN();
                actualCount = 0;
                actualColor = 0;
            }
        }
        else {
            if (actualCount == 0) {
                Led::off();
                ++actualCount;
            }
            else {
                Led::set(SteadyColor);
                actualCount = 0;
            }
        }
    }

private:
    inline static std::FixedVector<Color, MaximumColors> colors;
    inline static uint_NaN<uint8_t> count = [](){
            uint_NaN<uint8_t> v;
            v.setNaN();
            return v;
        }();
    inline static uint8_t actualColor = 0;
    inline static uint8_t actualCount = 0; 
};
