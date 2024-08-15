#pragma once

#include <etl/fixedvector.h>

template<typename Timer, typename PWM, typename Est, typename Config, typename Out = void>
struct KmFsm {
    using systemTimer = Timer;
    using pwm = PWM;
    using estimator = Est;
    using config = Config;

    enum class State : uint8_t {Idle, Start, Inc, Meas, Stop};
    enum class Event : uint8_t {NoEvent, Start};

    static inline constexpr External::Tick<systemTimer> measTicks{1000ms}; // fill the fft-buffer

    static inline void start() {
        if (mState == State::Idle) {
            mLastEvent = Event::Start;
        }
    }

    static inline bool process() {
        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::Idle:
            if (std::exchange(mLastEvent, Event::NoEvent) == Event::Start) {
                mState = State::Start;
            }
            break;
            case State::Start:
                mStateTick.on(measTicks, []{
                    mState = State::Inc;
                });
                break;
            case State::Inc:
                if (measureDuty > measureDutyMax) {
                    mState = State::Stop;
                }
                else {
                    mState = State::Meas;
                }
                break;
            case State::Meas:
                mStateTick.on(measTicks, []{
                    mState = State::Inc;
                });
                break;
            case State::Stop:
                mState = State::Idle;
                break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Idle:
                return true;
                break;
            case State::Start:
                IO::outl<Out>("# MeasRotStart");
                meKMs.clear();
                config::init();
                pwm::dir1();
                measureDuty = 0;
                pwm::duty(measureDuty);
                pwm::setSingleMode();
                break;
            case State::Inc:
                IO::outl<Out>("# MeasRotInc");
                saveResults();
                measureDuty += 100;
                pwm::duty(measureDuty);
                break;
            case State::Meas:
                IO::outl<Out>("# MeasRotMeas");
                break;
            case State::Stop:
                IO::outl<Out>("# MeasRotStop");
                measureDuty = 0;
                pwm::duty(measureDuty);
                calculate();
                break;
            }
        }
        return false;
    }
    static inline uint16_t getMeanEKm() {
        return meanEKm;
    }
    static inline void setRm(const float r) {
        IO::outl<Out>("# setRm: ", (uint16_t)(1000 * r));
        Rm = r;
    }
    template<typename S = Out>
    static inline void print() {
        for(const auto& k : meKMs){
            IO::outl<S>("# eKM: ", k);
        }
    }
    private:
    static inline void calculate() {
        uint32_t sum = 0;
        for(const auto& k : meKMs){
            sum += k;
        }
        meanEKm = sum / meKMs.size();
    }

    static inline void saveResults() {
        const uint16_t offset = estimator::simpleFFT();
        const uint32_t erpm = estimator::eRpmNoWindow();
        const float ubatt = estimator::uBattMean();
        const float duty = ((float)measureDuty / pwm::maxDuty());
        const float umotor = ubatt * duty;
        const float curr = estimator::currMean();

        const float umintern = umotor - curr * Rm;

        const uint16_t eKM = (float)erpm / umintern;
        if ((eKM >= minEKm) && (eKM <= maxEKm)) {
            meKMs.push_back(eKM);
        }
        IO::outl<Out>("# eRpm: ", erpm, " off: ", offset, " eKM: ", eKM, " um: ", (uint16_t)(umotor * 1000), " ui: ", (uint16_t)(umintern * 1000));
    }
    static inline uint16_t minEKm = 500;
    static inline uint16_t maxEKm = 5000;
    static inline float Rm = 0.0f;
    static inline Event mLastEvent = Event::NoEvent;
    static inline uint16_t measureDuty{};
    static inline uint16_t measureDutyMax = (0.9f * pwm::maxDuty());
    static inline State mState = State::Idle;
    static inline etl::FixedVector<uint16_t, 16> meKMs{};
    static inline uint16_t meanEKm = 0;
    static inline External::Tick<systemTimer> mStateTick;
};

