/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <algorithm>

template<typename T = int>
struct PID {
    using value_type = T;

    explicit PID(const T max, const T min, const T kp, const T ki, const T kd) :
        mMax{max}, mMin{min}, mKp{kp}, mKi{ki}, mKd{kd}, mW{kp + ki + kd}
    {}

    float process(const T error) {
        const T p = mKp * error;

        mIntegral = std::clamp(mIntegral + error, mMin, mMax);
        const T i = mKi * mIntegral;

        const T  deriv = error - mLastError;
        const T d = mKd * deriv;

        const T out = std::clamp((p + i + d) / (mKp + mKi + mKd), mMin, mMax);

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
    value_type mW{};
};
