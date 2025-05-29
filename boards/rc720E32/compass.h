#pragma once

#include "etl/algorithm.h"
#include "fastmath.h"
#include "tick.h"
#include "etl/event.h"
#include "eeprom.h"

using namespace std::literals::chrono_literals;

// todo: move to dsp.h
template<typename T = int32_t>
struct ExpMean {
    explicit ExpMean(const T n, const T d) : mN{n}, mD{d} {}
    inline T process(const T v) {
        mValue = (mValue * mN) / mD + (v * (mD - mN)) / mD;
        return mValue;
    }
    inline T value() const {
        return mValue;
    }
    inline void setN(const T n) {
        mN = n;
    }
    private:
    T mN = 100;
    T mD = 100;
    T mValue = 0;
};

// template<typename Dev, typename systemTimer, typename Client, typename Accelerometer, typename Storage, typename tp>
template<typename Config>
struct Compass {
    using magnetometer = Config::magnetometer;
    using acc = Config::accelerometer;
    using storage = Config::storage;
    using client = Config::client;
    using systemTimer = Config::timer;
    using tp = Config::tp;
    using debug = Config::debug;

    // all inits must be done in this time slot
    // todo: I2C magnetometers must specify the time for init
    static inline constexpr External::Tick<systemTimer> initTicks{1000ms};
    static inline constexpr External::Tick<systemTimer> idleTicks{10ms};
    static inline constexpr External::Tick<systemTimer> waitTicks{1ms};

    enum class State : uint8_t {Init, Idle, ReadMagneto, ReadAccel, CalibMagneto, WaitBetween};
    enum     class Event : uint8_t {None, StartCalibrate, StopCalibrate};

    // scale all integer operations (lack of floating point)
    static inline constexpr int16_t scale = 1000;
    static inline constexpr uint16_t ipi = FastMath::pi * scale;

    static inline constexpr auto& eeprom = storage::eeprom;

    static inline void init() {
        mXCalib = Calib{eeprom.compass_calib[0].mean, eeprom.compass_calib[0].d};
        mYCalib = Calib{eeprom.compass_calib[1].mean, eeprom.compass_calib[1].d};
        mZCalib = Calib{eeprom.compass_calib[2].mean, eeprom.compass_calib[2].d};
    }

    static inline void startCalibrate() {
        mXrange = MinMax{};
        mYrange = MinMax{};
        mZrange = MinMax{};
        client::start();
        mEvent = Event::StartCalibrate;
    }
    static inline void stopCalibrate() {
        if (mXCalib && mYCalib && mYCalib) {
            eeprom.compass_calib[0].mean = mXCalib.mean;
            eeprom.compass_calib[0].d    = mXCalib.d;
            eeprom.compass_calib[1].mean = mYCalib.mean;
            eeprom.compass_calib[1].d    = mYCalib.d;
            eeprom.compass_calib[2].mean = mZCalib.mean;
            eeprom.compass_calib[2].d    = mZCalib.d;
        }
        client::end();
        mEvent = Event::StopCalibrate;
    }
    static inline bool isCalibrating() {
        return (mState == State::CalibMagneto);
    }
    static inline void periodic() {
        magnetometer::periodic();
        acc::periodic();
    }
    static inline void ratePeriodic() {
        magnetometer::ratePeriodic();
        acc::ratePeriodic();
        const auto oldState = mState;
        ++mStateTicks;
        switch(mState) {
        case State::Init:
            mStateTicks.on(initTicks, []{
               mState = State::Idle;
            });
            break;
        case State::CalibMagneto:
            if (mEvent.is(Event::StopCalibrate)) {
                mState = State::Idle;
            }
            else {
                if (magnetometer::isIdle() && !magnetometer::isPendingEvent()) {
                    mXrange.add(magnetometer::x(), []{client::update();});
                    mYrange.add(magnetometer::y(), []{client::update();});
                    mZrange.add(magnetometer::z(), []{client::update();});
                    calculateCalibration();
                    magnetometer::startRead();
                }
            }
            break;
        case State::Idle:
            if (mEvent.is(Event::StartCalibrate)) {
                magnetometer::startRead();
                mState = State::CalibMagneto;
            }
            else {
                mStateTicks.on(idleTicks, []{
                    magnetometer::startRead();
                    mState = State::ReadMagneto;
                });
            }
            break;
        case State::ReadMagneto:
            if (magnetometer::isIdle() && !magnetometer::isPendingEvent()) {
                mX.process(magnetometer::x());
                mY.process(magnetometer::y());
                mZ.process(magnetometer::z());
                mState = State::WaitBetween;
            }
            break;
        case State::WaitBetween:
            mStateTicks.on(waitTicks, []{
                acc::startRead();
                mState = State::ReadAccel;
            });
            break;
        case State::ReadAccel:
            if (acc::isIdle() && !acc::isPendingEvent()) {
                mAccX.process(acc::accX());
                mAccY.process(acc::accY());
                mAccZ.process(acc::accZ());
                mState = State::Idle;
            }
            break;
        }
        if (oldState != mState) {
            mStateTicks.reset();
        }
    }
    static inline int16_t x() {
        return mXCalib.calc(mX.value());
    }
    static inline int16_t y() {
        return mYCalib.calc(mY.value());
    }
    static inline int16_t z() {
        return mZCalib.calc(mZ.value());
    }
    static inline int16_t accX() {
        return mAccX.value();
    }
    static inline int16_t accY() {
        return mAccY.value();
    }
    static inline int16_t accZ() {
        return mAccZ.value();
    }
    static inline int16_t a() {
        return FastMath::uatan2<scale, ipi>(mY.value(), mX.value());
    }
    static inline std::pair<int16_t, int16_t> pitchRoll_I() {
        const int16_t ax = accX();
        const int16_t ay = accY();
        const int16_t az = accZ();
        const uint32_t ax2 = ax * ax;
        const uint32_t ay2 = ay * ay;
        const uint32_t az2 = az * az;
        const int16_t sq_zy = FastMath::isqrt(az2 + ay2);
        const int16_t sq_zx = FastMath::isqrt(az2 + ax2);
        const int16_t pitch = -FastMath::atan2<scale, ipi>(ax, sq_zy);
        const int16_t roll  = FastMath::atan2<scale, ipi>(ay, sq_zx);
        if (az >= 0) {
            return {pitch, roll};
        }
        else {
            return {-pitch, -roll};
        }
    }
    // static inline std::pair<float, float> pitchRoll() {
    //     const float ax = accX();
    //     const float ay = accY();
    //     const float az = accZ();
    //     const float ax2 = ax * ax;
    //     const float ay2 = ay * ay;
    //     const float az2 = az * az;

    //     const float sq_zy = std::sqrt(az2 + ay2);
    //     const float sq_zx = std::sqrt(az2 + ax2);

    //     const float pitch = -std::atan2(ax, sq_zy);
    //     const float roll  = std::atan2(ay, sq_zx);

    //     if (az >= 0) {
    //         return {pitch, roll};
    //     }
    //     else {
    //         return {-pitch, -roll};
    //     }
    // }
    // // 452µs
    // static inline int16_t aComp() {
    //     tp::set();
    //     const float mag_x = x();
    //     const float mag_y = y();
    //     const float mag_z = z();

    //     const auto [p, r] = pitchRoll();

    //     const float X_h = mag_x * std::cos(p) + mag_z * std::sin(p);
    //     const float Y_h = mag_x * std::sin(r) * std::sin(p) + mag_y * std::cos(r) - mag_z * std::sin(r) * std::cos(p);
    //     const float azimuth = std::atan2(Y_h, X_h);

    //     tp::reset();
    //     return azimuth * 1000;
    // }

    // 34µs
    static inline int16_t aComp_I() {
        tp::set();
        const int16_t mag_x = x();
        const int16_t mag_y = y();
        const int16_t mag_z = z();

        const auto [p, r] = pitchRoll_I();

        const int16_t X_h = (mag_x * FastMath::cos<ipi, scale>(p) + mag_z * FastMath::sin<ipi, scale>(p)) / scale;
        const int16_t Y_h = ((mag_x * FastMath::sin<ipi, scale>(r) * FastMath::sin<ipi, scale>(p)) / scale +
                             mag_y * FastMath::cos<ipi, scale>(r) -
                             (mag_z * FastMath::sin<ipi, scale>(r) * FastMath::cos<ipi, scale>(p)) / scale) / scale;
        const int16_t azimuth = FastMath::atan2<scale, ipi>(Y_h, X_h);
        // const int16_t azimuth = FastMath::atan2<scale, 180>(Y_h, X_h);

        tp::reset();
        return azimuth;
    }
    // template<typename Debug>
    // static inline void debugInfo() {
    //     IO::outl<Debug>("# compass", " xmin: ", mXrange.min, " xmax: ", mXrange.max, " ymin: ", mYrange.min, " ymax: ", mYrange.max, " zmin: ", mZrange.min, " zmax: ", mZrange.max);
    //     mXCalib.template debugInfo<Debug>();
    //     mYCalib.template debugInfo<Debug>();
    //     mZCalib.template debugInfo<Debug>();
    // }
    private:
    struct MinMax {
        void add(const int16_t v, auto notify) {
            if (v < min) {
                min = v;
                notify();
            }
            if (v > max) {
                max = v;
                notify();
            }
        }
        int16_t min = 0;
        int16_t max = 0;
    };
    struct Calib {
        explicit Calib(const int16_t m, const int16_t d) : mean{m}, d{d} {}
        explicit Calib(const MinMax& mm = MinMax{}) {
            if ((mm.max - mm.min) > 2000) {
                mean = (mm.max + mm.min) / 2;
                d = (mm.max - mm.min) / 2;
            }
        }
        int16_t calc(const int16_t v) {
            if (d != 0) {
                const int16_t v0 = v - mean;
                return (v0 * int32_t(scale)) / d;
            }
            else {
                return v;
            }
        }
        // template<typename Debug>
        // void debugInfo() {
        //     IO::outl<Debug>("# mean: ", mean, " d: ", d);
        // }
        explicit operator bool() const {
            return (d != 0);
        }
        int16_t mean = 0;
        int16_t d = 0;
    };
    static inline void calculateCalibration() {
        mXCalib = Calib{mXrange};
        mYCalib = Calib{mYrange};
        mZCalib = Calib{mZrange};
        if (!(mXCalib && mYCalib && mYCalib)) {
            mXCalib = Calib{};
            mYCalib = Calib{};
            mZCalib = Calib{};
        }
    }
    static inline MinMax mXrange;
    static inline MinMax mYrange;
    static inline MinMax mZrange;
    static inline Calib mXCalib;
    static inline Calib mYCalib;
    static inline Calib mZCalib;

    static inline ExpMean mX{90, 100};
    static inline ExpMean mY{90, 100};
    static inline ExpMean mZ{90, 100};
    static inline ExpMean mAccX{90, 100};
    static inline ExpMean mAccY{90, 100};
    static inline ExpMean mAccZ{90, 100};

    static inline etl::Event<Event> mEvent;
    static inline State mState = State::Init;
    static inline External::Tick<systemTimer> mStateTicks;
};
