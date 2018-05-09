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
#include <array>
#include <ctype.h>
#include "container/stringbuffer.h"
#include "container/pgmstring.h"

template<typename Leds, uint8_t Rows, uint8_t Columns, uint8_t TextLength, typename Font, uint8_t Space = 1>
class ShiftDisplay final {
    ShiftDisplay() = delete;
public:
    typedef typename Leds::color_type Color;
    
    template<uint8_t L>    
    static void set(const StringBuffer<L>& string) {
        uint8_t position = 0;
        for(uint8_t i = 0; i < L; ++i) {
            uint8_t c = string[i];
            if (iscntrl(c)) {
                break;
            }
            position = insertBitmap(position, mFont[c]);
            for(uint8_t col = 0; col < Space; ++col) {
                mBitmap[position++] = 0;
            }
        }
    }
    template<typename C, C... Cs>
    static void set(const PgmString<C, Cs...>& pgm) {
        uint8_t position = 0;
        for(uint8_t i = 0; i < pgm.size; ++i) {
            uint8_t c = pgm_read_byte(&pgm.data[i]);
            if (iscntrl(c)) {
                break;
            }
            position = insertBitmap(position, mFont[c]);
            for(uint8_t col = 0; col < Space; ++col) {
                mBitmap[position++] = 0;
            }
        }
    }

    static void write() {
        for(uint8_t x = 0; x < Columns; ++x) {
            uint8_t mask = 1;
            for(uint8_t y = 0; y < 8; ++y) {
                uint8_t index = 0;
                if ((y % 2) == 0) {
                    index = 21 + y * 11 - x;
                }
                else {
                    index = 11 + y * 11 + x;
                }
                if (mBitmap[x + mStartInBitmap] & mask) {
                    Leds::template set<false>(index, Color{255});
                }
                else {
                    Leds::template set<false>(index, Color{0});
                }
                mask <<= 1;
            }
        }
        Leds::write();
    }
    static void shift() {
        mStartInBitmap = (mStartInBitmap + 1) % mBitmap.size;
    }
    static void reset() {
        mStartInBitmap = 0;
    }
    static void clear() {
        std::fill(std::begin(mBitmap), std::end(mBitmap), 0);
    }

    static const Font& font() {
        return mFont;
    }
private:
    static uint8_t insertBitmap(uint8_t position, const typename Font::Char& fontChar) {
        for(uint8_t col = 0; col < Font::Width; ++col) {
            if (position >= mBitmap.size) {
                break;
            }
            if (auto bits = fontChar[col]; bits != 0) {
                mBitmap[position++] = bits;
            }
        }
        return position;
    }

    inline static std::array<uint8_t, TextLength * (Font::Width + Space)> mBitmap = {};
    inline static const Font mFont;
    inline static uint8_t mStartInBitmap = 0;
};
