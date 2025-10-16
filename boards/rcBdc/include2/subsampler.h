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
        using store = devs::store;

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
        static inline void invert(const bool b) { // and offset
            mInvert = b;
            pga::useOffset(b);
        }
        static inline void isr() {
            // const float currFiltered = iirFilter.process(mInvert ? (4095 - adc::mData[0]) : adc::mData[0]);
            const float currFiltered = iirFilter.process(mInvert ? (4095 - adc::values()[0]) : adc::values()[0]);
            // mMeanVoltage.process(adc::mData[1]);
            mMeanVoltage.process(adc::values()[1]);

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
        static inline void reset() {
            gainIndex = 0;
            pga::gain(gainIndex);
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
        static inline volatile bool mInvert{false};
        static inline constexpr std::array<uint8_t, 7> gainFactor{1, 2, 4, 8, 16, 32, 64};
        static inline Dsp::Butterworth::LowPass<6, volatile float> iirFilter;
        static inline Dsp::ExpMean<void> mCurrMeanADC{0.001};
        static inline volatile Dsp::ExpMean<void> mCurrMean{0.01};
        static inline volatile Dsp::ExpMean<void> mMeanVoltage{0.0001};
    };
}
