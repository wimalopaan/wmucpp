#pragma once

#include <optional>
#include <utility>

#include "cmsis.h"
#include "eeprom.h"

namespace Dsp {
    template<uint16_t Size, typename Source>
    struct FFTEstimator {
        using source = Source;
        using store = Source::store;

        static inline constexpr uint16_t size = Size;

        static inline void init() {
            arm_rfft_fast_init_f32(&fftInstance, Size);
            for(uint16_t i = 0; i < (2 * windowWidth); ++i) {
                mWindowPlot[i].second = window[i];
            }
        }
        static inline uint16_t simpleFFT() {
            // atomic copy of data
            __disable_irq();
            std::copy(std::begin(Source::data()), std::end(Source::data()), &samples[0]);
            __enable_irq();

            if (timeWindow == 1) {
                for(uint16_t i = 0; i < samples.size(); ++i) {
                    samples[i] *= blackmanWindow[i];
                }
            }
            else if (timeWindow == 2) {
                for(uint16_t i = 0; i < samples.size(); ++i) {
                    samples[i] *= hannWindow[i];
                }
            }

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
            // find absolute maximum after the leftmost minimum (exclude signal time-average)
            const uint16_t offset = minIndex + 1;
            arm_max_f32(&mMagnitude[offset], mMagnitude.size() - offset, &maxValue, &maxIndex);
            maxIndex += offset;
            return offset;
        }
        static inline uint32_t eRpmNoWindow() {
            return index2Erpm(maxIndex);
        }
        static inline uint32_t eRpm() {
            return index2Erpm(maxIndexWeighted);
        }
        static inline float uBattMean() {
            return Source::devs::adc2Voltage(Source::meanVoltage());
        }
        static inline float currMean() {
            return Source::devs::adc2Current(Source::currMean());
        }
        static inline void update(const auto duty) {
            const uint16_t offset = simpleFFT();
            if (duty) {
                const float Ubatt = Source::devs::adc2Voltage(Source::meanVoltage());
                const uint32_t erpmMax = Ubatt * (mDir1 ? mEKM.dir1 : mEKM.dir2);
                const uint16_t pmax = eRpm2index(erpmMax);

                const float absDuty = (duty.toInt() >= 0) ? duty.toInt() : -duty.toInt();
                const float absDutyRel = absDuty / ((duty.Upper - duty.Lower) / 2);
                const float curr = Source::devs::adc2Current(Source::currMean());
                const float udiff = curr * (mDir1 ? mRM.dir1 : mRM.dir2);
                const float umotor = Ubatt * absDutyRel;
                const float umeff = umotor - udiff;

                const float indexCutoff = absDutyRel * (Size / store::eeprom.n_fsample);

                const float rpm = std::max(std::min(pmax * umeff / Ubatt, (float)(Size/2 - 1)), 0.0f);

                // the following only from left to right
                const uint16_t windowLeft = std::max(rpm - windowWidth, 1.0f);
                const uint16_t windowRight = std::min(rpm + windowWidth, (float)(Size/2 - 1));

                for(uint16_t i = windowLeft; i <= windowRight; ++i) {
                    mMagnitudeWeighted[i] = mMagnitude[i] * wf(i, rpm);
                }

                arm_max_f32(&mMagnitudeWeighted[windowLeft], windowRight - windowLeft + 1, &maxValueWeighted, &maxIndexWeighted);
                maxIndexWeighted += windowLeft;

                // for(uint16_t i = 0; i < Size / 2; ++i) {
                //     mMagnitudeWeighted[i] = mMagnitude[i] * wf(i, rpm);
                // }

                // arm_max_f32(&mMagnitudeWeighted[offset], mMagnitudeWeighted.size() - offset, &maxValueWeighted, &maxIndexWeighted);
                // maxIndexWeighted += offset;

                mMaxWeighted[0].first = std::max((int)maxIndexWeighted - 10, 0);
                mMaxWeighted[0].second = maxValueWeighted * 0.9;
                mMaxWeighted[1].first = maxIndexWeighted;
                mMaxWeighted[1].second = maxValueWeighted;
                mMaxWeighted[2].first = std::min((int)maxIndexWeighted + 10, Size/2 - 1);
                mMaxWeighted[2].second = maxValueWeighted * 0.9;

                mRpmPos[0].first = std::max((int)rpm - 10, 0);
                mRpmPos[0].second = maxValue * 0.9;
                mRpmPos[1].first = rpm;
                mRpmPos[1].second = maxValue;
                mRpmPos[2].first = std::min((int)rpm + 10, Size/2 - 1);
                mRpmPos[2].second = maxValue * 0.9;

                mIndexCutoff[0].first = indexCutoff;
                mIndexCutoff[0].second = 0;
                mIndexCutoff[1].first = indexCutoff;
                mIndexCutoff[1].second = maxValue;

                mWindowPlot.clear();
                for(uint16_t i = windowLeft; i < windowRight; ++i) {
                    std::pair<uint16_t, float> c;
                    c.first = i;
                    c.second = maxValue * wf(i, rpm);
                    mWindowPlot.push_back(c);
                }
            }
        }
        static inline const auto& magnitude() {
            return mMagnitude;
        }
        static inline const auto& windowPlot() {
            return mWindowPlot;
        }
        static inline const auto& maxWeighted() {
            return mMaxWeighted;
        }
        static inline const auto eRpmWeighted() {
            return index2Erpm(maxIndexWeighted);
        }
        static inline const auto& rpmPos() {
            return mRpmPos;
        }
        static inline const auto& magnitudeWeighted() {
            return mMagnitudeWeighted;
        }
        static inline const auto& indexCutoff() {
            return mIndexCutoff;
        }
        static inline void setEKm(const Directional<uint16_t> k) {
            mEKM = k;
        }
        static inline void setRm(const Directional<float> r) {
            mRM = r;
        }
        static inline void dir1(const bool d) {
            mDir1 = d;
        }
        static inline void windowFunction(const uint8_t w) {
            timeWindow = w;
        }
        static inline uint16_t eRpm2index(const uint32_t erpm) {
            return (erpm * Size) / (60U * Source::samplingFrequency());
        }
        static inline uint32_t index2Erpm(const uint16_t i) {
            return (60U * (uint32_t)i * Source::samplingFrequency()) / Size;
        }
        private:
        static inline bool mDir1 = true;
        static inline Directional<uint16_t > mEKM{1875, 1875}; // electrical-Upm/V
        static inline Directional<float> mRM{7.9f, 7.9f}; // resistance (Ohm)

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

        static inline std::array<std::pair<uint16_t, float>, 3> mMaxWeighted;
        static inline std::array<std::pair<uint16_t, float>, 3> mRpmPos;
        static inline std::array<std::pair<uint16_t, float>, 2> mIndexCutoff;

        static inline constexpr uint16_t searchFirstMinIndexMax = 100;
        static inline constexpr uint16_t windowWidth = 75;
        static inline constexpr auto window = []{
            std::array<float, 2 * windowWidth> w{};
            for(uint16_t i = 0; i < (2 * windowWidth); ++i) {
                const float c = cos(std::numbers::pi_v<float> * (float)(i - windowWidth) / (2 * windowWidth));
                // w[i] = c;
                w[i] = c * c;
            }
            return w;
        }();
        static inline uint8_t timeWindow = 1;
        static inline constexpr auto blackmanWindow = []{
            // Blackmann-Nuttall
            const float a0 = 0.3635819;
            const float a1 = 0.4891775;
            const float a2 = 0.1365995;
            const float a3 = 0.0106411;
            std::array<float, Size> w{};
            for(uint16_t i = 0; i < Size; ++i) {
                w[i] = a0;
                w[i] -= a1 * cos((2 * std::numbers::pi_v<float> * i) / (Size - 1));
                w[i] += a2 * cos((4 * std::numbers::pi_v<float> * i) / (Size - 1));
                w[i] -= a3 * cos((6 * std::numbers::pi_v<float> * i) / (Size - 1));
            }
            return w;
        }();
        static inline constexpr auto hannWindow = []{
            std::array<float, Size> w{};
            for(uint16_t i = 0; i < Size; ++i) {
                w[i] = 0.5f * (1.0f - cos((2 * std::numbers::pi_v<float> * i) / (Size - 1)));
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
        static inline etl::FixedVector<std::pair<uint16_t, float>, 2 * windowWidth> mWindowPlot;
    };
}
