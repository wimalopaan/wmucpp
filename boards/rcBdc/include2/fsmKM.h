#pragma once

#include <etl/fixedvector.h>

template<typename Timer, typename PWM, typename Est, typename Config, typename Out = void>
struct KmFsm {
    using systemTimer = Timer;
    using pwm = PWM;
    using estimator = Est;
    using config = Config;
    using subSampler = Est::source;
    using devs = subSampler::devs;
    using offset = devs::offset;
    using store = devs::store;

    enum class State : uint8_t {Idle,
                                Start, Inc, Meas, Stop, Slowdown,
                               };
    enum class Event : uint8_t {NoEvent, Start};

    static inline constexpr External::Tick<systemTimer> measTicks{1000ms}; // fill the fft-buffer
    static inline constexpr External::Tick<systemTimer> slowdownTicks{50ms};

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
                mDir1 = true;
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
                mState = State::Slowdown;
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
        case State::Slowdown:
            mStateTick.on(slowdownTicks, []{
                measureDuty -= 20;
                if (measureDuty > 0) {
                    pwm::duty(measureDuty);
                }
                else {
                    measureDuty = 0;
                    pwm::duty(measureDuty);
                    mState = State::Stop;
                }
            });
            break;
        case State::Stop:
            if (mDir1) {
                mDir1 = false;
                mState = State::Start;
            }
            else {
                mState = State::Idle;
            }
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Idle:
                IO::outl<Out>("# KM Idle");
                return true;
                break;
            case State::Start:
                IO::outl<Out>("# KM MeasRotStart");
                config::init();
                subSampler::reset();
                lastERpm = 0;
                if (mDir1) {
                    meKMs_dir1.clear();
                    if constexpr (!std::is_same_v<offset, void>) {
#ifdef TEST_C1
#else
                        offset::set();
#endif
                        subSampler::invert(true);
                    }
                    pwm::dir1();
                }
                else {
                    meKMs_dir2.clear();
                    if constexpr (!std::is_same_v<offset, void>) {
#ifdef TEST_C1
#else
                        offset::reset();
#endif
                        subSampler::invert(false);
                    }
                    pwm::dir2();
                }
                measureDuty = 0;
                pwm::duty(measureDuty);
                pwm::setSingleMode();
                break;
            case State::Inc:
                IO::outl<Out>("# KM MeasRotInc");
                saveResults();
                measureDuty += 100;
                pwm::duty(measureDuty);
                break;
            case State::Meas:
                IO::outl<Out>("# KM MeasRotMeas");
                break;
            case State::Slowdown:
                IO::outl<Out>("# KM SlowDown");
                break;
            case State::Stop:
                IO::outl<Out>("# KM MeasRotStop");
                calculate();
                break;
            }
        }
        return false;
    }
    static inline bool isDir1() {
        return mDir1;
    }
    static inline Directional<uint16_t> getMeanEKm() {
        return {meanEKm1, meanEKm2};
    }
    static inline void setRm(const Directional<float> r) {
        IO::outl<Out>("# setRm: ", (uint16_t)(1000 * r.dir1));
        Rm = r;
    }
    template<typename S = Out>
    static inline void print() {
        for(uint8_t i = 0; i < meKMs_dir1.size(); ++i) {
            IO::outl<S>("# eKM: ", meKMs_dir1[i], " ", meKMs_dir2[i]);
        }
    }
    private:
    static inline uint16_t mean(const auto& c) {
        uint32_t sum = 0;
        uint16_t num = 0;
        for(const auto& e: c) {
            if (e > 0) {
                sum += e;
                ++num;
            }
        }
        if (num > 0) {
            return sum / num;
        }
        else {
            return 0;
        }
        // const auto sum = std::accumulate(std::begin(c), std::end(c), 0);
        // return sum / c.size();
    }
    static inline void calculate() {
        meanEKm1 = mean(meKMs_dir1);
        meanEKm2 = mean(meKMs_dir2);
    }

    static inline void saveResults() {
        const uint16_t offset = estimator::simpleFFT();
        const uint32_t erpm = estimator::eRpmNoWindow();
        const float ubatt = estimator::uBattMean();
        const float duty = ((float)measureDuty / pwm::maxDuty());
        const float umotor = ubatt * duty;
        const float curr = estimator::currMean();

        const float umintern = umotor - curr * (mDir1 ? Rm.dir1 : Rm.dir2);
        const uint16_t eKM = (float)erpm / umintern;

        const float indexCutoff = duty * (estimator::size / store::eeprom.n_fsample);
        const uint32_t erpmCutoff = estimator::index2Erpm(indexCutoff);

        if ((eKM >= minEKm) && (eKM <= erpmCutoff)) {
            if (mDir1) {
                if (erpm > lastERpm) {
                    meKMs_dir1.push_back(eKM);
                    lastERpm = erpm;
                }
            }
            else {
                if (erpm > lastERpm) {
                    meKMs_dir2.push_back(eKM);
                    lastERpm = erpm;
                }
            }
        }
        IO::outl<Out>("# KM eRpm: ", erpm, " off: ", offset, " eKM: ", eKM, " um: ", (uint16_t)(umotor * 1000), " ui: ", (uint16_t)(umintern * 1000), " g: ", subSampler::gain(), " erpm c: ", erpmCutoff);
    }
    static inline bool mDir1 = true;
    static inline uint16_t minEKm = 500;
    static inline Directional Rm{0.0f, 0.0f};
    static inline Event mLastEvent = Event::NoEvent;
    static inline int16_t measureDuty{};
    static inline uint16_t measureDutyMax = (0.9f * pwm::maxDuty()); // parameter
    static inline State mState = State::Idle;
    static inline etl::FixedVector<uint16_t, 16> meKMs_dir1{};
    static inline etl::FixedVector<uint16_t, 16> meKMs_dir2{};
    static inline uint16_t lastERpm = 0;
    static inline uint16_t meanEKm1 = 0;
    static inline uint16_t meanEKm2 = 0;
    static inline External::Tick<systemTimer> mStateTick;
};

