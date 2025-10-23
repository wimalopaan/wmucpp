/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

template<typename T = float>
struct PID {
    using value_type = T;

    explicit PID(const float max, const float min, const float kp, const float ki, const float kd) :
        mMax{max}, mMin{min}, mKp{kp}, mKi{ki}, mKd{kd}
    {}

    float process(const float set, const float meas) {
        const float error = set - meas;
        const float p = mKp * error;

        mIntegral += error * mKi;
		mIntegral = std::clamp(mIntegral, mMin, mMax);
		
        const float i = mIntegral;

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
