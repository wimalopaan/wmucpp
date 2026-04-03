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

struct FastMath {
    static inline constexpr float pi = std::numbers::pi_v<float>;
    static inline constexpr float pi_2 = std::numbers::pi_v<float> / 2.0f;
    static inline constexpr float pi_4 = std::numbers::pi_v<float> / 4.0f;

    // [0, Res] : Res -> 2pi
    template<uint16_t Max = 820, uint16_t Res = 4096>
    static inline uint16_t uatan2(const int16_t y, const int16_t x) {
        static constexpr auto atan_lut = [] consteval {
            std::array<uint16_t, Max + 1> lut;
            for(uint16_t i = 0; i < lut.size(); ++i) {
                lut[i] = (std::atan((float)i / Max) / pi_4) * Res / 8;
            }
            lut[Max] = Res / 8;
            return lut;
        }();

        if (x == 0) {
            if (y > 0) {
                return Res / 4;
            }
            else if (y < 0) {
                return (3 * Res) / 4;
            }
            else { // y == 0
                return 0;
            }
        }
        else { // x != 0
            const uint16_t xa = std::abs(x);
            const uint16_t ya = std::abs(y);
            if (x > 0) {
                if (y >= 0) { // Q1
                    if (xa >= ya) {
                        return atan_lut[(ya * Max) / xa];
                    }
                    else {
                        return (Res / 4) - atan_lut[(xa * Max) / ya];
                    }
                }
                else { // Q4
                    if (xa >= ya) {
                        return Res - atan_lut[(ya * Max) / xa];
                    }
                    else {
                        return ((3 * Res) / 4) + atan_lut[(xa * Max) / ya];
                    }
                }
            }
            else {
                if (y >= 0) { // Q2
                    if (xa >= ya) {
                        return (Res / 2) - atan_lut[(ya * Max) / xa];
                    }
                    else {
                        return (Res / 4) + atan_lut[(xa * Max) / ya];
                    }
                }
                else { // Q3
                    if (xa >= ya) {
                        return (Res / 2) + atan_lut[(ya * Max) / xa];
                    }
                    else {
                        return ((3 * Res) / 4) - atan_lut[(xa * Max) / ya];
                    }
                }
            }
        }
    }
    // [-Res, Res] : Res -> pi
    template<uint16_t Max = 820, uint16_t Res = 2000>
    static inline int16_t atan2(const int16_t y, const int16_t x) {
        static constexpr auto atan_lut = []consteval{
            std::array<uint16_t, Max + 1> lut;
            for(uint16_t i = 0; i < lut.size(); ++i) {
                lut[i] = (std::atan((float)i / Max) / pi_4) * Res / 4;
            }
            lut[Max] = Res / 4;
            return lut;
        }();

        if (x == 0) {
            if (y > 0) {
                return Res / 2;
            }
            else if (y < 0) {
                return -Res / 2;
            }
            else { // y == 0
                return 0;
            }
        }
        else { // x != 0
            const uint16_t xa = std::abs(x);
            const uint16_t ya = std::abs(y);
            if (x > 0) {
                if (y >= 0) { // Q1
                    if (xa >= ya) {
                        return atan_lut[(ya * Max) / xa];
                    }
                    else {
                        return (Res / 2) - atan_lut[(xa * Max) / ya];
                    }
                }
                else { // Q4
                    if (xa >= ya) {
                        return -atan_lut[(ya * Max) / xa];
                    }
                    else {
                        return -(Res / 2) + atan_lut[(xa * Max) / ya];
                    }
                }
            }
            else {
                if (y >= 0) { // Q2
                    if (xa >= ya) {
                        return Res - atan_lut[(ya * Max) / xa];
                    }
                    else {
                        return (Res / 2) + atan_lut[(xa * Max) / ya];
                    }
                }
                else { // Q3
                    if (xa >= ya) {
                        return -Res + atan_lut[(ya * Max) / xa];
                    }
                    else {
                        return -(Res / 2) - atan_lut[(xa * Max) / ya];
                    }
                }
            }
        }
    }

    // [-Max, Max] -> [-Res, Res]
    template<uint16_t Max = (pi * 1000), uint16_t Res = 1000>
    static inline int16_t cos(const int16_t x) {
        static constexpr auto cos_lut = [] consteval {
            std::array<int16_t, Max + 1> lut;
            for(uint16_t i = 0; i < lut.size(); ++i) {
                lut[i] = std::cos((i * pi) / Max) * Res;
            }
            lut[Max] = -Res;
            return lut;
        }();

        const uint16_t xa = std::abs(x);
        return cos_lut[xa];
    }
    // [-Max, Max] -> [-Res, Res]
    template<uint16_t Max = (pi * 1000), uint16_t Res = 1000>
    static inline int16_t sin(const int16_t x) {
        static constexpr auto sin_lut = [] consteval {
            std::array<int16_t, Max + 1> lut;
            for(uint16_t i = 0; i < lut.size(); ++i) {
                lut[i] = std::sin((i * pi) / Max) * Res;
            }
            lut[Max] = 0;
            return lut;
        }();

        if (x >= 0) {
            return sin_lut[x];
        }
        else {
            return -sin_lut[-x];
        }
    }
    static inline unsigned usqrt4(const unsigned val) {
        unsigned a, b;
        if (val < 2) return val; /* avoid div/0 */
        a = val / 3;
        b = val / a; a = (a+b) /2;
        b = val / a; a = (a+b) /2;
        b = val / a; a = (a+b) /2;
        b = val / a; a = (a+b) /2;
        return a;
    }
    static inline unsigned isqrt(unsigned long val) {
        unsigned long temp, g=0, b = 0x8000, bshft = 15;
        do {
            if (val >= (temp = (((g << 1) + b)<<bshft--))) {
                g += b;
                val -= temp;
            }
        } while (b >>= 1);
        return g;
    }
};


