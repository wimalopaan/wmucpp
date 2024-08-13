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
            for(uint16_t i = 0; i < (2 * windowWidth); ++i) {
                windowPlot[i].second = window[i];
            }
        }
        static inline uint16_t simpleFFT() {
            __disable_irq();
            std::copy(std::begin(Source::data()), std::end(Source::data()), &samples[0]);
            __enable_irq();

            arm_rfft_fast_f32(&fftInstance, &samples[0], &fft[0], 0); // fft[0] (real) : dc-offset
            arm_cmplx_mag_f32(&fft[0], &mMagnitude[0], mMagnitude.size()); // interleaved (real, complex) input, normal output

            float minValue = mMagnitude[0];
            minIndex = 0;
            for(uint16_t i = 1; i < searchFirstMinIndexMax; ++i) {
                if (mMagnitude[i] < minValue) {
                    minValue = mMagnitude[i];
                    mMagnitude[i - 1] = 0;
                    minIndex = i;
                }
                else {
                    break;
                }
            }
            const uint16_t offset = minIndex + 1;
            float maxValue{};
            arm_max_f32(&mMagnitude[offset], mMagnitude.size() - offset, &maxValue, &maxIndex);
            maxIndex += offset;
            return offset;
        }
        static inline uint32_t eRpmNoWindow() {
            return index2Erpm(maxIndex);
        }
        static inline float uBattMean() {
            return Source::devs::adc2Voltage(Source::meanVoltage());
        }
        static inline float currMean() {
            return Source::devs::adc2Current(Source::currMean());
        }
        static inline void update(const auto p) {
            const uint16_t offset = simpleFFT();

            if (p) {
                const float Ubatt = Source::devs::adc2Voltage(Source::meanVoltage());
                const uint32_t erpmMax = Ubatt * mEKM;
                const uint16_t pmax = eRpm2index(erpmMax);

                const int16_t px = (p.toInt() >= 0) ? p.toInt() : -p.toInt();
                const float curr = Source::devs::adc2Current(Source::currMean());
                const float udiff = curr * mRM;
                const float umotor = ((Ubatt * px) / ((p.Upper - p.Lower) / 2) - udiff);

                const float rpm = pmax * umotor / Ubatt;

                const uint16_t windowLeft = std::max(rpm - windowWidth, 1.0f);
                const uint16_t windowRight = std::min(rpm + windowWidth, (float)(Size/2 - 1));

                for(uint16_t i = 0; i < Size / 2; ++i) {
                    mMagnitudeWeighted[i] = mMagnitude[i] * wf(i, rpm);
                }

                arm_max_f32(&mMagnitudeWeighted[offset], mMagnitudeWeighted.size() - offset, &maxValueWeighted, &maxIndexWeighted);
                maxIndexWeighted += offset;

                for(uint16_t i = 0; i < mMagnitudeWeighted.size(); ++i) {
                    mMagnitudeWeighted[i] = std::min(mMagnitudeWeighted[i], maxValue * 5.0f);
                }

                maxWeighted[0].first = std::max((int)maxIndexWeighted - 10, 0);
                maxWeighted[0].second = maxValueWeighted * 0.9;
                maxWeighted[1].first = maxIndexWeighted;
                maxWeighted[1].second = maxValueWeighted;
                maxWeighted[2].first = std::min((int)maxIndexWeighted + 10, Size/2);
                maxWeighted[2].second = maxValueWeighted * 0.9;

                rpmPos[0].first = std::max((int)rpm - 10, 0);
                rpmPos[0].second = maxValue * 0.9;
                rpmPos[1].first = rpm;
                rpmPos[1].second = maxValue;
                rpmPos[2].first = std::min((int)rpm + 10, Size/2);
                rpmPos[2].second = maxValue * 0.9;

                windowPlot.clear();
                for(uint16_t i = windowLeft; i < windowRight; ++i) {
                    std::pair<uint16_t, float> c;
                    c.first = i;
                    c.second = maxValue * wf(i, rpm);
                    windowPlot.push_back(c);
                }
            }
        }
        static inline const auto& magnitude() {
            return mMagnitude;
        }
        static inline const auto& magnitudeWeighted() {
            return mMagnitudeWeighted;
        }
    private:
        static inline uint16_t eRpm2index(const uint32_t erpm) {
            return (erpm * Size) / (60U * Source::samplingFrequency());
        }
        static inline uint32_t index2Erpm(const uint16_t i) {
            return (60U * (uint32_t)i * Source::samplingFrequency()) / Size;
        }
        static inline float mEKM{1875.0f}; // electrical-Upm/V
        static inline float mRM{7.9f}; // resistance (Ohm)

        static inline uint32_t minIndex{0};
        static inline float maxValue{};
        static inline float maxValueWeighted{};
        static inline uint32_t maxIndex{};
        static inline uint32_t maxIndexWeighted{};
        static inline arm_rfft_fast_instance_f32 fftInstance{};
        static inline std::array<float, Size> samples{};
        static inline std::array<float, Size> fft{};
        static inline std::array<float, Size / 2> mMagnitude{};
        static inline std::array<float, Size / 2> mMagnitudeWeighted{};
        public:
        static inline std::array<std::pair<uint16_t, float>, 3> maxWeighted;
        static inline std::array<std::pair<uint16_t, float>, 3> rpmPos;

        static inline constexpr uint16_t searchFirstMinIndexMax = 100;
        static inline constexpr uint16_t windowWidth = 75;
        static inline constexpr auto window = []{
            std::array<float, 2 * windowWidth> w{};
            for(uint16_t i = 0; i < (2 * windowWidth); ++i) {
                const float c = cos(std::numbers::pi_v<float> * (float)(i - windowWidth) / (2 * windowWidth));
                w[i] = c * c;
            }
            return w;
        }();
        static inline constexpr float wf(const uint16_t index, const uint16_t mid) {
            const int16_t i = (index - mid) + windowWidth;
            if (i < 0) {
                return 0;
            }
            else if (i > (2 * windowWidth)) {
                return 0;
            }
            return window[i];
        }
        static inline etl::FixedVector<std::pair<uint16_t, float>, 2 * windowWidth> windowPlot;
    };
}
