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

    static inline float atan_scalar_approximation(const float x) {
        static constexpr float a1  =  0.99997726f;
        static constexpr float a3  = -0.33262347f;
        static constexpr float a5  =  0.19354346f;
        static constexpr float a7  = -0.11643287f;
        static constexpr float a9  =  0.05265332f;
        // static constexpr float a11 = -0.01172120f;

        const float x_sq = x * x;
        // return x * (a1 + x_sq * (a3 + x_sq * (a5 + x_sq * (a7 + x_sq * (a9 + x_sq * a11)))));
        return x * (a1 + x_sq * (a3 + x_sq * (a5 + x_sq * (a7 + x_sq * (a9)))));
        // return x * (a1 + x_sq * (a3 + x_sq * a5));
    }
    static inline float atan2(const float y, const float x) {
        const float abs_y = std::fabs(y) + 1e-10f;      // kludge to prevent 0/0 condition
        const float r = (x - std::copysign(abs_y, x)) / (abs_y + std::fabs(x));
        float angle = pi_2 - std::copysign(pi_4, x);

        angle -= atan_scalar_approximation(r);
        // angle += (0.1963f * r * r - 0.9817f) * r;
        return std::copysign(angle, y);
    }
    // static inline float atan2(const float y, const float x) {
    //     // Ensure input is in [-1, +1]
    //     const bool swap = fabs(x) < fabs(y);
    //     const float atan_input = (swap ? x : y) / (swap ? y : x);

    //     // Approximate atan
    //     float res = atan_scalar_approximation(atan_input);

    //     // If swapped, adjust atan output
    //     res = swap ? (atan_input > 0.0f ? pi_2 : -pi_2) - res : res;
    //     // Adjust quadrants
    //     if      (x >= 0.0f && y >= 0.0f) {}                     // 1st quadrant
    //     else if (x <  0.0f && y >= 0.0f) { res =  pi + res; } // 2nd quadrant
    //     else if (x <  0.0f && y <  0.0f) { res = -pi + res; } // 3rd quadrant
    //     else if (x >= 0.0f && y <  0.0f) {}                     // 4th quadrant

    //     return res;
    // }

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


