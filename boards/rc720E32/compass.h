#pragma once

#include "etl/algorithm.h"
#include "fastmath.h"
#include "tick.h"

using namespace std::literals::chrono_literals;

struct ExpMeanInt {
    explicit ExpMeanInt(const int32_t n, const int32_t d) : mN{n}, mD{d} {}
    inline int32_t process(const int32_t v) {
        mValue = (mValue * mN) / mD + (v * (mD - mN)) / mD;
        return mValue;
    }
    inline int32_t value() const {
        return mValue;
    }
    inline void setN(const int32_t n) {
        mN = n;
    }
    private:
    int32_t mN = 100;
    int32_t mD = 100;
    int32_t mValue = 0;
};
template<typename Dev, typename systemTimer, typename Client>
struct Compass {
    using dev = Dev;

    static inline constexpr External::Tick<systemTimer> updateTicks{20ms};

    static inline void startCalibrate() {
        mXrange = MinMax{};
        mYrange = MinMax{};
        mZrange = MinMax{};
        mCalibrating = true;
        Client::start();
    }
    static inline void stopCalibrate() {
        mCalibrating = false;
        Client::end();
    }
    static inline void periodic() {
        dev::periodic();
    }
    static inline void ratePeriodic() {
        dev::ratePeriodic();
        (++mTicks).on(updateTicks, []{
            mX.process(dev::x());
            mY.process(dev::y());
            mZ.process(dev::z());
            if (mCalibrating) {
                mXrange.add(dev::x(), []{Client::update();});
                mYrange.add(dev::y(), []{Client::update();});
                mZrange.add(dev::z(), []{Client::update();});
            }
            dev::startRead();
        });
    }
    static inline int16_t x() {
        return mX.value();
    }
    static inline int16_t y() {
        return mY.value();
    }
    static inline int16_t z() {
        return mZ.value();
    }
    static inline int16_t a() {
        return FastMath::uatan2<1000, 360>(mY.value(), mX.value());
    }
    template<typename Debug>
    static inline void debugInfo() {
        IO::outl<Debug>("# compass", " xmin: ", mXrange.min, " xmax: ", mXrange.max, " ymin: ", mYrange.min, " ymax: ", mYrange.max, " zmin: ", mZrange.min, " zmax: ", mZrange.max);
    }
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
        void operator+=(const int16_t v) {
            if (v < min) min = v;
            if (v > max) max = v;
        }
        int16_t min = 0;
        int16_t max = 0;
    };
    static inline bool mCalibrating = false;
    static inline MinMax mXrange;
    static inline MinMax mYrange;
    static inline MinMax mZrange;
    static inline ExpMeanInt mX{90, 100};
    static inline ExpMeanInt mY{90, 100};
    static inline ExpMeanInt mZ{90, 100};
    static inline External::Tick<systemTimer> mTicks;

};
