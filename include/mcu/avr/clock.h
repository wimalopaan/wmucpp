/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <stdint.h>
#include <time.h>
#include "mcu/avr8.h"
#include "mcu/concepts.h"
#include "util/time.h"

namespace AVR {

template<const std::hertz& TickRate, MCU::Pin DebugPin = void, typename MCU = DefaultMcuType>
class Clock {
public:
    static constexpr const std::hertz& tickRate = TickRate;
    static constexpr auto mcuClock = getBaseAddr<typename MCU::Clock>;

    static inline void init() {}
    static inline void start() {
        mDelta = 0;
        mMeasurementDuration = 0;
        mSecondsStart = 0;
        mSecondsStart = 0;
    }
    static inline void referenceTime(const DateTime::TimeTm& t) {
        auto tm = t.tm();
        auto tt = mktime(&tm);
        if (mSecondsStart == 0) {
            mSecondsStart = tt;
        }
        else {
            mMeasurementDuration = std::wideSeconds{tt - mSecondsStart};
            mSecondsStart = tt;
            uint32_t expectedTicks = mMeasurementDuration * tickRate;            
            mDelta = mTicks - expectedTicks;
        }
        mTicks = 0;
    }
    static inline std::wideSeconds lastMeasurementDuration() {
        return mMeasurementDuration;
    }
    static inline std::optional<std::wideSeconds> actualMeasurementDuration(const DateTime::TimeTm& t) {
        if (mSecondsStart == 0) {
            return {};
        }
        auto tm = t.tm();
        auto tt = mktime(&tm);
        return std::wideSeconds{tt - mSecondsStart};        
    }

    static inline int32_t delta() {
        return mDelta;
    }
    static inline uint8_t calibration() {
        return *mcuClock()->osccal;
    }
    static inline void adjust(int8_t delta) {
        *mcuClock()->osccal += delta;
    }
    static inline void rateProcess() {
        if constexpr(!std::is_same<DebugPin, void>::value) {
            DebugPin::toggle();
        }
        ++mTicks;
    }
    static inline uint32_t ticks() {
        return mTicks;
    }
private:
    inline static int32_t mDelta = 0;
    inline static uint32_t mTicks = 0;
    inline static time_t mSecondsStart = 0;
    inline static std::wideSeconds mMeasurementDuration{0};
    
    inline static constexpr uint32_t mSecondsPerMinute = 60;
public:
    inline static constexpr std::array<std::wideSeconds, 3> mIntervalls = {3 * mSecondsPerMinute, 60 * mSecondsPerMinute, 
                24 * 60 * mSecondsPerMinute};
};

}