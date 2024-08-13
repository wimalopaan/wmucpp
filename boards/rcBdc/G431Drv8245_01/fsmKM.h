#pragma once

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
                if (measureDuty > (pwm::maxDuty() - 200)) {
                    // if (measureDuty > (pwm::maxDuty() / 2)) {
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
                break;
            }
        }
        return false;
    }
    static inline void setRm(const float r) {
        IO::outl<Out>("# setRm: ", (uint16_t)(1000 * r));
        Rm = r;
    }

    private:
    static inline void saveResults() {
        const uint16_t offset = estimator::simpleFFT();
        const uint32_t erpm = estimator::eRpmNoWindow();
        const float ubatt = estimator::uBattMean();
        const float duty = ((float)measureDuty / pwm::maxDuty());
        const float umotor = ubatt * duty;
        const float curr = estimator::currMean();

        const float umintern = umotor - curr * Rm;

        const float eKM = (float)erpm / umintern;
        IO::outl<Out>("# eRpm: ", erpm, " off: ", offset, " eKM: ", (uint32_t)eKM, " um: ", (uint16_t)(umotor * 1000), " ui: ", (uint16_t)(umintern * 1000));
    }
    static inline float Rm = 0.0f;
    static inline Event mLastEvent = Event::NoEvent;
    static inline uint16_t measureDuty{};
    static inline State mState = State::Idle;
    static inline External::Tick<systemTimer> mStateTick;
};

