#pragma once

#include <cstdint>
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <numbers>
#include <numeric>

#include "etl/algorithm.h"

namespace Dsp {
    struct IQ {
        float i{};
        float q{};
    };

    struct FCut {
        float f1{};
        float f2{};
    };
    
    struct IQ_Bands {
        IQ low{};
        IQ high{};
    };
    
    constexpr double sinc(const double x) {
        if (x == 0.0) return 1.0;
        return sin(std::numbers::pi * x) / (std::numbers::pi * x);
    }
    constexpr inline void crc16(uint16_t& crc, const uint8_t value) {
        constexpr uint16_t crc_polynome = 0x1021;
        crc = crc ^ (((uint16_t)value) << 8);
        for(uint8_t i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ crc_polynome;
            }
            else {
                crc = (crc << 1);
            }
        }
    }

    template<typename Config>
    struct ExpMax {
        constexpr explicit ExpMax(const float f) : f{f} {}
        constexpr float process(const float v) {
            max *= (1.0 - f);
            if (v > max) max = v;
            return max;
        }
    private:
        float f{};
        float max{};
    };

    template<typename Config>
    struct ExpMean {
        constexpr explicit ExpMean(const float f = 0.0f) : f{f} {}
        constexpr float process(const float v) {
            mean = f * v + (1.0 - f) * mean;
            return mean;
        }
        constexpr void set(const float m) {
            mean = m;
        }
        constexpr float value() const volatile {
            return mean;
        }
    private:
        float f{};
        float mean{};
    };

    template<typename Config>
    struct StepLimiter {
        constexpr uint16_t process(const uint16_t v) {
            if (v < (last - Config::maxImpulseDelta)) {
                last = std::max(last - Config::maxImpulseDelta, int(Config::nMinPulse));
            }            
            else if (v > (last + Config::maxImpulseDelta)) {
                last = std::min(last + Config::maxImpulseDelta, int(Config::nMaxPulse));
            }
            else {
                last = v;
            }            
            return last;
        }
        constexpr uint16_t value() const {
            return last;
        }
    private:
        uint16_t last{(Config::nMaxPulse + Config::nMinPulse) / 2};
    };

    
    template<typename Config>
    struct HighPass {
        constexpr float process(const float v) {
            last = Config::alphaHighPass * v + (1 - Config::alphaHighPass) * last;
            return v - last;
        }
    private:
        float last{};
    };
    
    
    template<uint8_t L>
    struct Convolve {
        constexpr Convolve(const std::array<float, L>& c) : coeff{c} {}
        constexpr float process(const float v) {
            buffer[in] = v;
            const float result = [&]{
                float sum{0};
                uint8_t i{0};
                for(int8_t k = in; k >= 0; ++i, --k) {
                    sum += coeff[i] * buffer[k];
                }
                for(int8_t k = L - 1; i < L; ++i, --k) {
                    sum += coeff[i] * buffer[k];
                }
                return sum;
            }();
            in += 1;
            if (in == L) in = 0;
            return result;
        }
    private:
        uint8_t in{0};
        std::array<float, L> buffer{};    
        std::array<float, L> coeff{};    
    };    
    
    
    
    struct BandPass;
    
    template<uint8_t Length, typename Type = BandPass, float fs = 48000, FCut ff_l = FCut{4500, 5500}, FCut ff_h = FCut{9500, 10500}>
    struct FirFilterMulti;
    
    template<uint8_t L, float fs, FCut fl, FCut fh>
    struct FirFilterMulti<L, BandPass, fs, fl, fh> {
        constexpr IQ_Bands process(const IQ v) {
            buffer[in] = v;
            const IQ_Bands result = [&]{
                IQ_Bands sum{};
                uint_fast8_t i{0};
                for(int_fast8_t k = in; k >= 0; ++i, --k) {
                    sum.low.i += coeff_l[i] * buffer[k].i;
                    sum.low.q += coeff_l[i] * buffer[k].q;
                    sum.high.i += coeff_h[i] * buffer[k].i;
                    sum.high.q += coeff_h[i] * buffer[k].q;
                }
                for(int_fast8_t k = L - 1; i < L; ++i, --k) {
                    sum.low.i += coeff_l[i] * buffer[k].i;
                    sum.low.q += coeff_l[i] * buffer[k].q;
                    sum.high.i += coeff_h[i] * buffer[k].i;
                    sum.high.q += coeff_h[i] * buffer[k].q;
                }
                return sum;
            }();
            if (++in == L) in = 0;
            return result;
        }
    private:
        uint_fast8_t in{0};
        std::array<IQ, L> buffer{};    
        static inline constexpr auto calculate(const float f1, const float f2) {
            std::array<double, L> window;
            constexpr double alpha   = 0.54;
            constexpr double beta    = 0.46;
            for(int i = 0; i < L; i++) {
                window[i] = alpha - beta * cos(2.0 * std::numbers::pi * i / (L - 1));
            }
            std::array<float, L> cc;
            double df1 = f1 / fs;
            double df2 = f2 / fs;
            for(int i = 0; i < L; i++) {
                double n = i - ((L - 1) / 2.0); 
                cc[i] = 2.0 * df1 * sinc(2.0 * df1 * n) - 2.0 * df2 * sinc(2.0 * df2 * n);
                cc[i] *= window[i];
            }
            
            double fc = (f1 + f2) / 2.0;
            double gain_c = 0.0;
            double gain_r = 0.0;
            double gain_i = 0.0;
            for(int i = 0; i < L; i++) {
                gain_r += cos(2 * std::numbers::pi * (fc / fs) * i) * cc[i];
                gain_i += sin(2 * std::numbers::pi * (fc / fs) * i) * cc[i];
            }
            gain_c = sqrt(gain_r * gain_r + gain_i * gain_i);
            for(int i = 0; i < L; i++) {
                cc[i] /= gain_c;
            }
            return cc;
        }
        static inline constexpr auto coeff_l = calculate(fl.f1, fl.f2);
        static inline constexpr auto coeff_h = calculate(fh.f1, fh.f2);
    };
    
    template<uint16_t Length, typename Type = BandPass, float fs = 48000, float f1 = 4500, float f2 = 5500>
    struct FirFilter;
    
    template<uint16_t L, float fs, float f1, float f2>
    struct FirFilter<L, BandPass, fs, f1, f2> {
        constexpr float process(const float v) {
            buffer[in] = v;
            
//            const float result = [&]<auto... II>(std::index_sequence<II...>){
//                                 float sum{};
//                                 uint8_t k = in;
//                                 (((sum += coeff[II] * buffer[k]), (k == 0) ? (k = L - 1) : --k), ...);
                                 
//                                 return sum;
//            }(std::make_index_sequence<L>{});
            
//            const float result = [&]{
//                float sum{};
//                for(uint8_t i{0}, k = in; i < L; ++i) {
//                    sum += coeff[i] * buffer[k];
//                    if (k == 0) {
//                        k = L - 1;
//                    }
//                    else {
//                        --k;
//                    }
//                }
//                return sum;
//            }();
            
//            buffer[in] = v;
//            buffer[in + L] = v;
//            const float result = std::inner_product(std::begin(buffer) + in, 
//                                                    std::end(buffer) + in + L, 
//                                                    std::begin(coeff), 0.0f);
//            in = (in == 0) ? (L - 1) : (in - 1);
            
            // Multi-Filter i,q mit mehreren Impulsantworten
            
            const float result = [&]{
                float sum{0};
                uint8_t i{0};
                for(int8_t k = in; k >= 0; ++i, --k) {
                    sum += coeff[i] * buffer[k];
                }
                for(int8_t k = L - 1; i < L; ++i, --k) {
                    sum += coeff[i] * buffer[k];
                }
                return sum;
            }();
            in += 1;
            if (in == L) in = 0;
            return result;
        }
    private:
        uint8_t in{0};
        std::array<float, L> buffer{};    
//        std::array<float, 2 * L> buffer{};    
    public:
        static inline constexpr auto coeff = []{
            std::array<double, L> window;
            constexpr double alpha   = 0.54;
            constexpr double beta    = 0.46;
            for(int i = 0; i < L; i++) {
                window[i] = alpha - beta * cos(2.0 * std::numbers::pi * i / (L - 1));
            }
            std::array<float, L> cc;
            constexpr double df1 = f1 / fs;
            constexpr double df2 = f2 / fs;
            for(int i = 0; i < L; i++) {
                double n = i - ((L - 1) / 2.0); 
                cc[i] = 2.0 * df1 * sinc(2.0 * df1 * n) - 2.0 * df2 * sinc(2.0 * df2 * n);
                cc[i] *= window[i];
            }
            
            constexpr double fc = (f1 + f2) / 2.0;
            double gain_c = 0.0;
            double gain_r = 0.0;
            double gain_i = 0.0;
            for(int i = 0; i < L; i++) {
                gain_r += cos(2 * std::numbers::pi * (fc / fs) * i) * cc[i];
                gain_i += sin(2 * std::numbers::pi * (fc / fs) * i) * cc[i];
            }
            gain_c = sqrt(gain_r * gain_r + gain_i * gain_i);
            for(int i = 0; i < L; i++) {
                cc[i] /= gain_c;
            }
            return cc;
        }();
    };

    template<uint16_t L>
    struct Max {
        constexpr float process(const float v) {
            buffer[in] = v;
            const float max = etl::maximum(buffer);
            if (++in == L) in = 0;
            return max;
        }            
    private:
        std::array<float, L> buffer{};
        uint_fast16_t in{0};
    };
    
    
    
    
}
