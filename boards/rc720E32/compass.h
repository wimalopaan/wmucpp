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

template<typename Dev, typename systemTimer>
struct Compass {
    using dev = Dev;

    static inline constexpr External::Tick<systemTimer> updateTicks{20ms};

    static inline void periodic() {
        dev::periodic();
    }
    static inline void ratePeriodic() {
        dev::ratePeriodic();
        (++mTicks).on(updateTicks, []{
            mX.process(dev::x());
            mY.process(dev::y());
            mZ.process(dev::z());
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
    // static inline int16_t a2() {
    //     return std::atan2(mY.value(), mX.value()) * 1000;
    // }
    private:
    static inline ExpMeanInt mX{90, 100};
    static inline ExpMeanInt mY{90, 100};
    static inline ExpMeanInt mZ{90, 100};
    static inline External::Tick<systemTimer> mTicks;

};
