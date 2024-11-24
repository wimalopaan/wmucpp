#pragma once

template<typename T = float>
struct PID {
    using value_type = T;

    explicit PID(const float max, const float min, const float kp, const float ki, const float kd) :
        mMax{max}, mMin{min}, mKp{kp}, mKi{ki}, mKd{kd}
    {}

    float process(const float set, const float meas) {
        const float error = set - meas;
        const float p = mKp * error;

        mIntegral += error;
        const float i = mKi * mIntegral;

        const float  deriv = error - mLastError;
        const float d = mKd * deriv;

        const float out = std::max(std::min(p + i + d, mMax), mMin);

        mLastError = error;
        return out;
    }

    private:
    value_type mIntegral{};
    value_type mLastError{};
    value_type mMax{};
    value_type mMin{};
    value_type mKp{};
    value_type mKi{};
    value_type mKd{};
};
