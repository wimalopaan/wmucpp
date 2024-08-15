#pragma once

template<typename Timer, typename PWM, typename Identifier, typename Config, typename Out = void>
struct RLFsm {
    using systemTimer = Timer;
    using pwm = PWM;
    using identifier = Identifier;
    using config = Config;

    enum class State : uint8_t {Idle, Start, Inc, Meas, Stop};
    enum class Event : uint8_t {NoEvent, Start};

    static inline constexpr External::Tick<systemTimer> measTicks{300ms};

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
            if (measureDuty > (pwm::maxDuty() / 2)) {
                mState = State::Stop;
            }
            else {
                mState = State::Meas;
            }
            break;
        case State::Meas:
            // wenn Sättigung erreicht
            if (identifier::maxEst.value() > 4000) {
                mState = State::Stop;
            }
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
                IO::outl<Out>("# MeasStart");
                measurements = RLMeasurements<volatile float>{};
                measureDuty = 0;
                pwm::duty(measureDuty);
                pwm::setMultiMode();
                pwm::pwm(config::fPwmIdentify);
                identifier::reset();
                break;
            case State::Inc:
                IO::outl<Out>("# MeasInc");
                // save RL values
                if ((actualMeasCount_dir1 > minMeasuresPerLevel) && (actualMeasCount_dir2 > minMeasuresPerLevel)) {
                    RL<> m1;
                    uint16_t c1;
                    RL<> m2;
                    uint16_t c2;
                    __disable_irq();
                    m1 = actualMeas_dir1;
                    m2 = actualMeas_dir2;
                    c1 = actualMeasCount_dir1;
                    c2 = actualMeasCount_dir2;
                    actualMeasCount_dir1 = 0;
                    actualMeasCount_dir2 = 0;
                    actualMeas_dir1 = RL<>{};
                    actualMeas_dir2 = RL<>{};
                    __enable_irq();
                    m1 /= c1;
                    m2 /= c2;
                    measurements += {m1, m2};
                }
                measureDuty += config::dutyIdentifyInc;
                pwm::duty(measureDuty);
                break;
            case State::Meas:
                IO::outl<Out>("# MeasMeas");
                break;
            case State::Stop:
                IO::outl<Out>("# MeasStop");
                identifier::reset();
                measureDuty = 0;
                pwm::duty(measureDuty);
                break;
            }
        }
        return false;
    }
    static inline void addMeasurement(const RL<> rl, const bool dir) {
        if (dir) {
            actualMeas_dir1 += rl;
            actualMeasCount_dir1 += 1;
        }
        else {
            actualMeas_dir2 += rl;
            actualMeasCount_dir2 += 1;
        }
    }
    template<typename S = Out>
    static inline void print() {
        for(uint8_t i = 0; i < measurements.step; ++i) {
            IO::outl<S>("# step: ", i, " Rm1: ", (uint16_t)(1000 * measurements.meanRL_dir1[i].Rm), " Rm2: ", (uint16_t)(1000 * measurements.meanRL_dir2[i].Rm));
            IO::outl<S>("# step: ", i, " Lm1: ", (uint16_t)(1000 * measurements.meanRL_dir1[i].Lm), " Lm2: ", (uint16_t)(1000 * measurements.meanRL_dir2[i].Lm));
        }
    }
    static inline float getLastRm(const bool dir1 = true) {
        const uint8_t last = measurements.step - 1;
        if (dir1) {
            return measurements.meanRL_dir1[last].Rm;
        }
        else {
            return measurements.meanRL_dir2[last].Rm;
        }
    }
    static inline float getLastLm(const bool dir1 = true) {
        const uint8_t last = measurements.step - 1;
        if (dir1) {
            return measurements.meanRL_dir1[last].Lm;
        }
        else {
            return measurements.meanRL_dir2[last].Lm;
        }
    }
    private:
    static inline constexpr uint16_t minMeasuresPerLevel = 10;
    static inline RL<volatile float> actualMeas_dir1{};
    static inline volatile uint16_t actualMeasCount_dir1{0};
    static inline RL<volatile float> actualMeas_dir2{};
    static inline volatile uint16_t actualMeasCount_dir2{0};
    static inline RLMeasurements<volatile float> measurements;

    static inline Event mLastEvent = Event::NoEvent;
    static inline uint16_t measureDuty{};
    static inline State mState = State::Idle;
    static inline External::Tick<systemTimer> mStateTick;
};

