#pragma once

#include <optional>
#include <utility>

#include "cmsis.h"

namespace Dsp {
    template<typename Devices, uint16_t Size = 2048>
    struct SubSampler {
        using devs = Devices;
        using adc = devs::adc;
        using pga = devs::pga;

        using tp2 = devs::tp2;
        static inline void cutoff(const uint16_t f) {
            iirFilter.fc(f);
        }
        static inline void pwm(const uint16_t f) {
            iirFilter.fs(f);
        }
        static inline void subSampleFactor(const uint16_t f) {
            factor = f;
        }
        static inline void isr() {
            const float currFiltered = iirFilter.process(adc::mData[0]);
            mMeanVoltage.process(adc::mData[1]);

            sampleCounter += 1;
            if (sampleCounter == factor) {
                sampleCounter = 0;
                const float currSubsampled = currFiltered / gainFactor[gainIndex];
                if (index == mData.size()) {
                    index = 0;
                }
                mData[index] = currSubsampled;
                index = index + 1;

                mCurrMeanADC.process(currFiltered); // ADC overflow
                mCurrMean.process(currSubsampled); // real value

                if (mCurrMeanADC.value() > 4095.0f * (2.0f / 3.0f)) {
                    if (gainIndex > 0) {
                        gainIndex -= 1;
                        pga::gain(gainIndex);
                        mCurrMeanADC.set(mCurrMeanADC.value() / 2);
                    }
                }
                else if (mCurrMeanADC.value() < 4095.0f * (1.0f / 3.0f)) {
                    if (gainIndex < 6) {
                        gainIndex += 1;
                        pga::gain(gainIndex);
                        mCurrMeanADC.set(mCurrMeanADC.value() * 2);
                    }
                }
            }
        }
        static inline void whenValueAvailable(auto f) {
            bool flag = false;
            float v;
            __disable_irq();
            if (!mData.empty()) {
                flag = true;
                mData.pop_front(v);
            }
            __enable_irq();
            if (flag) {
                f(v);
            }
        }
        static inline float currMean() {
            return mCurrMean.value();
        }
        static inline float meanVoltage() {
            return mMeanVoltage.value();
        }
        static inline float currMeanADC() {
            return mCurrMeanADC.value();
        }
        static inline uint8_t gain() {
            return gainFactor[gainIndex];
        }
        static inline const auto& data() {
            return mData;
        }
        static inline float samplingFrequency() {
            return iirFilter.fs() / factor;
        }
    private:
        static inline std::array<volatile float, Size> mData;
        static inline volatile uint16_t index{0};
        static inline volatile uint16_t sampleCounter{0};
        static inline volatile uint16_t factor{10};
        static inline volatile uint8_t gainIndex{0};
        static inline constexpr std::array<uint8_t, 7> gainFactor{1, 2, 4, 8, 16, 32, 64};
        static inline Dsp::Butterworth::LowPass<6, volatile float> iirFilter;
        static inline Dsp::ExpMean<void> mCurrMeanADC{0.001};
        static inline volatile Dsp::ExpMean<void> mCurrMean{0.01};
        static inline volatile Dsp::ExpMean<void> mMeanVoltage{0.0001};
    };

#if 0
    template<typename Devices, uint16_t Size = 2048>
    struct SubSampler {
        using devs = Devices;
        using adc = devs::adc;
        using pga = devs::pga;

        using tp2 = devs::tp2;
        static inline void cutoff(const uint16_t f) {
            iirFilter.fc(f);
        }
        static inline void pwm(const uint16_t f) {
            iirFilter.fs(f);
        }
        static inline void subSampleFactor(const uint16_t f) {
            factor = f;
        }
        static inline void isr() {
            const auto x = iirFilter.process(adc::mData[0]);
            sampleCounter += 1;
            if (sampleCounter == factor) {
                sampleCounter = 0;
                const float xg = x / gainFactor[gain];
                data.push_back(xg);

                mMean.process(x);

                if (mMean.value() > 4095.0f * (2.0f / 3.0f)) {
                    if (gain > 0) {
                        gain -= 1;
                        pga::gain(gain);
                        mMean.set(mMean.value() / 2);
                    }
                }
                else if (mMean.value() < 4095.0f * (1.0f / 3.0f)) {
                    if (gain < 5) {
                        gain += 1;
                        pga::gain(gain);
                        mMean.set(mMean.value() * 2);
                    }
                }
            }
        }
        static inline void whenValueAvailable(auto f) {
            bool flag = false;
            float v;
            __disable_irq();
            if (!data.empty()) {
                flag = true;
                data.pop_front(v);
            }
            __enable_irq();
            if (flag) {
                f(v);
            }
        }
        // private:
        static inline etl::FiFo<volatile float, Size> data;
        static inline volatile uint16_t sampleCounter{0};
        static inline volatile uint16_t factor{10};
        static inline volatile uint8_t gain{0};
        static inline constexpr std::array<uint8_t, 6> gainFactor{2, 4, 8, 16, 32, 64};
        static inline Dsp::Butterworth::LowPass<6, volatile float> iirFilter;
        // static inline Dsp::ExpMax<void> mMax{0.0001};
        static inline Dsp::ExpMean<void> mMean{0.001};

    };
#endif
}
