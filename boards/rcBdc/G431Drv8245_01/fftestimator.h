#pragma once

#include <optional>
#include <utility>

#include "cmsis.h"

namespace Dsp {
    template<uint16_t Size, typename Source>
    struct FFTEstimator {
        static inline void init() {
            // arm_status status = arm_rfft_fast_init_f32(&fftInstance, Size);
            arm_rfft_fast_init_f32(&fftInstance, Size);
            for(uint16_t i = 0; i < (2 * width); ++i) {
                d3[i].second = window[i];
            }
        }
        static inline void update(const auto p) {
            __disable_irq();
            std::copy(std::begin(Source::data), std::end(Source::data), &samples[0]);
            __enable_irq();

            arm_rfft_fast_f32(&fftInstance, &samples[0], &fft[0], 0); // fft[0] (real) : dc-offset
            arm_cmplx_mag_f32(&fft[0], &magnitude[0], magnitude.size()); // interleaved (real, complex) input, normal output

            float minValue = magnitude[0];
            minIndex = 0;
            for(uint16_t i = 1; i < 100; ++i) {
                if (magnitude[i] < minValue) {
                    minValue = magnitude[i];
                    minIndex = i;
                }
                else {
                    break;
                }
            }
            uint16_t offset = minIndex + 1;
            float maxValue{};
            arm_max_f32(&magnitude[offset], magnitude.size() - offset, &maxValue, &maxIndex);
            maxIndex += offset;

            if (p) {
                // const uint16_t pmax = 300; // fftindex T60S
                const uint16_t pmax = 400; // fftindex 500e
                const float Ubatt = Source::devs::adc2Voltage(Source::mMeanVoltage.value());

                const uint16_t px = p.toInt() + 1000; // 0 ... 2000

                const float curr = Source::devs::adc2Current(Source::mCurrMean.value());

                // const float udiff = curr * 0.83f; // TS60
                const float udiff = curr * 0.58f; // 500
                // const float udiff = curr * 0.1f; // Torque 900
                const float umotor = ((Ubatt * px) / 2000 - udiff);

                const float rpm = pmax * umotor / Ubatt;

                const uint16_t a = std::max(rpm - width, 1.0f);
                const uint16_t b = std::min(rpm + width, (float)(Size/2 - 1));

                for(uint16_t i = 0; i < Size / 2; ++i) {
                    if (i < minIndex) {
                        magnitude2[i] = 0;
                        magnitude[i] = 0;
                    }
                    else {
                        magnitude2[i] = magnitude[i];
                        magnitude[i] *= wf(i, rpm);
                    }
                }
                uint16_t offset = minIndex + 1;
                arm_max_f32(&magnitude[offset], magnitude.size() - offset, &maxValue, &maxIndex2);
                maxIndex2 += offset;

                for(uint16_t i = 0; i < magnitude2.size(); ++i) {
                    magnitude2[i] = std::min(magnitude2[i], maxValue * 5.0f);
                }

                d1[0].first = std::max((int)maxIndex2 - 10, 0);
                d1[0].second = maxValue * 0.9;
                d1[1].first = maxIndex2;
                d1[1].second = maxValue;
                d1[2].first = std::min((int)maxIndex2 + 10, Size/2);
                d1[2].second = maxValue * 0.9;


                d2[0].first = std::max((int)rpm - 10, 0);
                d2[0].second = maxValue * 0.9;
                d2[1].first = rpm;
                d2[1].second = maxValue;
                d2[2].first = std::min((int)rpm + 10, Size/2);
                d2[2].second = maxValue * 0.9;

                d3.clear();
                for(uint16_t i = a; i < b; ++i) {
                    std::pair<uint16_t, float> c;
                    c.first = i;
                    c.second = maxValue * wf(i, rpm);
                    d3.push_back(c);
                }
            }
        }
    // private:
        static inline uint32_t minIndex{0};
        static inline float maxValue{};
        static inline uint32_t maxIndex{};
        static inline uint32_t maxIndex2{};
        static inline arm_rfft_fast_instance_f32 fftInstance{};
        static inline std::array<float, Size> samples{};
        static inline std::array<float, Size> fft{};
        static inline std::array<float, Size / 2> magnitude{};
        static inline std::array<float, Size / 2> magnitude2{};
        public:
        static inline std::array<std::pair<uint16_t, float>, 3> d1;
        static inline std::array<std::pair<uint16_t, float>, 3> d2;

        static inline constexpr uint16_t width = 75;
        static inline constexpr auto window = []{
            std::array<float, 2 * width> w{};
            for(uint16_t i = 0; i < (2 * width); ++i) {
                const float c = cos(std::numbers::pi_v<float> * (float)(i - width) / (2 * width));
                w[i] = c * c;
            }
            return w;
        }();
        static inline constexpr float wf(const uint16_t index, const uint16_t mid) {
            const int16_t i = (index - mid) + width;
            if (i < 0) {
                return 0;
            }
            else if (i > (2 * width)) {
                return 0;
            }
            return window[i];
        }
        static inline etl::FixedVector<std::pair<uint16_t, float>, 2 * width> d3;
    };

#if 0
    template<uint16_t Size, typename Source>
    struct FFTEstimator {
        static inline void init() {
            arm_status status = arm_rfft_fast_init_f32(&fftInstance, Size);
        }
        static inline void update() {
            __disable_irq();
            std::copy(std::begin(Source::data.data()), std::end(Source::data.data()), &samples[0]);
            __enable_irq();

            arm_rfft_fast_f32(&fftInstance, &samples[0], &fft[0], 0); // fft[0] (real) : dc-offset
            arm_cmplx_mag_f32(&fft[0], &magnitude[0], Size / 2); // interleaved (real, complex) input, normal output

            float minValue = magnitude[0];
            minIndex = 0;
            for(uint16_t i = 1; i < 100; ++i) {
                if (magnitude[i] < minValue) {
                    minValue = magnitude[i];
                    minIndex = i;
                }
                else {
                    break;
                }
            }

            uint16_t offset = minIndex + 1;
            float maxValue{};
            arm_max_f32(&magnitude[offset], Size / 2 - offset, &maxValue, &maxIndex);
            maxIndex += offset;
        }
        // private:
        static inline uint32_t minIndex{0};
        static inline float maxValue{};
        static inline uint32_t maxIndex{};
        static inline arm_rfft_fast_instance_f32 fftInstance{};
        static inline std::array<float, Size> samples{};
        static inline std::array<float, Size> fft{};
        static inline std::array<float, Size / 2> magnitude{};
    };
#endif
}
