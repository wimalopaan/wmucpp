#pragma once

template<typename T = float>
struct RL {
    T Rm;
    T Lm;
    template<typename U>
    void operator+=(const RL<U> rhs) {
        Rm += rhs.Rm;
        Lm += rhs.Lm;
    }
    template<typename U>
    void operator=(const RL<U> rhs) {
        Rm = rhs.Rm;
        Lm = rhs.Lm;
    }
    const RL& operator/=(const float d) {
        Rm /= d;
        Lm /= d;
        return *this;
    }
};

template<typename T = float, uint8_t Size = 20>
struct RLMeasurements {
    void operator+=(const std::pair<RL<>, RL<>>& m) {
        if (step < Size) {
            meanRL_dir1[step] = m.first;
            meanRL_dir2[step] = m.second;
            ++step;
        }
    }
    uint16_t step{};
    std::array<RL<T>, Size> meanRL_dir1{};
    std::array<RL<T>, Size> meanRL_dir2{};
};

template<typename Devices, typename Out = void>
struct BdcIdentifier {
    using devs = Devices;
    using adc = devs::adc;
    using pga = devs::pga;

    enum class State : uint8_t {Wait, Measure, Measure2};

    static inline void reset() {
        pga::gain(0); // follower-mode
        wait();
    }

    static inline void wait() {
        mState = State::Wait;
    }

    static inline void startMeasure() {
        if (mState == State::Wait) {
            voltageMean = 0.0f;
            sampleCount = 0;
            mState = State::Measure;
        }
    }
    static inline void sampleIsr() {
        lastAdc = adc::mData[0];
        const float current = devs::adc2Current(lastAdc);
        if (mState == State::Measure) {
            voltageMean += devs::adc2Voltage(adc::mData[1]);
            if (current > 0) { // warten bis Werte kommen
                devs::tp2::set();
                rle = Statistics::RLEstimator<volatile float>{1.0f, 0.0f, current};
                rle.process(current);
                sampleCount = sampleCount + 1;
                mState = State::Measure2;
                devs::tp2::reset();
            }
        }
        else if (mState == State::Measure2) {
            devs::tp2::set();
            voltageMean += devs::adc2Voltage(adc::mData[1]);
            rle.process(current);
            sampleCount = sampleCount + 1;
            devs::tp2::reset();
        }
    }
    static inline std::optional<RL<>> calculateIsr() {
        if ((mState == State::Measure2) && (sampleCount > 0)) {
            mState = State::Wait;
            maxEst.process(lastAdc);
            voltageMean /= sampleCount;
            return evaluate(voltageMean);
        }
        return {};
    }

    private:
    static inline std::optional<RL<>> evaluate(const float Um) {
        if (const auto ab = rle.compute()) {
            const float a = -ab->second / ab->first;
            const float b = ab->first;
            const float Rm = Um / a;
            const float Lm = (-a / b) / Um;
            if constexpr (!std::is_same_v<Out, void>) {
                IO::outl<Out>("# Um: ", (uint16_t)(1000 * Um), " Rm: ", (uint16_t)(1000 * Rm), " Lm: ", (uint16_t)(1000 * Lm));
            }
            return RL<>{Rm, Lm};
        }
        return {};
    }

    static inline Statistics::RLEstimator<volatile float> rle{1.0f, 0.0, 0.0};
    static inline volatile uint16_t sampleCount = 0;
    static inline volatile uint16_t lastAdc = 0;
    static inline volatile float voltageMean{};
    static inline volatile State mState{State::Wait};
    public:
    static inline volatile Dsp::ExpMean<void> maxEst{0.1};
};

