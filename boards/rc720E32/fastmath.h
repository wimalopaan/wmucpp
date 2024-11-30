#pragma once

struct FastMath {
    static inline constexpr float pi = std::numbers::pi_v<float>;
    static inline constexpr float pi_2 = std::numbers::pi_v<float> / 2.0f;
    static inline constexpr float pi_4 = std::numbers::pi_v<float> / 4.0f;

    template<auto Max = 820, auto Res = 4096>
    static inline uint16_t uatan2(const int16_t y, const int16_t x) {
        static constexpr auto atan_lut = []{
            static constexpr uint16_t max = 820;
            std::array<uint16_t, max + 1> lut;
            for(uint16_t i = 0; i < lut.size(); ++i) {
                lut[i] = (std::atan((float)i / max) / pi_4) * Res / 8;
            }
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
};


