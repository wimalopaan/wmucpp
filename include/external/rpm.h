/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <cstdint>
#include <limits>
#include "mcu/avr/isr.h"
#include "mcu/avr/mcutimer.h"
#include "util/types.h"
#include "util/bits.h"
#include "util/disable.h"
#include "units/physical.h"

template<typename MCUTimer>
constexpr uint16_t calculateRpm(const std::RPM& rpmMin, const std::RPM& rpmMax, uint32_t tvMin = 50) {
    using pBits = typename MCUTimer::mcu_timer_type::template PrescalerBits<MCUTimer::number>;
    auto p = AVR::Util::prescalerValues(pBits::values);
    auto sortedPRow = ::Util::sort(p, std::greater<typename AVR::PrescalerPair<typename MCUTimer::tccrb_type>::scale_type>()); // absteigend

    for(const auto& p : sortedPRow) {
        if (p > 0) {
            uint32_t tvalMax = (Config::fMcu.value * 60) / (p * rpmMin.value()); 
            uint32_t tvalMin = (Config::fMcu.value * 60) / (p * rpmMax.value()); 
            if ((tvalMax < std::numeric_limits<typename MCUTimer::value_type>::max()) && (tvalMin > tvMin)) {
                return p;
            }
        }
    }
    return 0;
}

template<typename InterruptSource, typename MCUTimer, const std::RPM& MaxRpm, const std::RPM& MinRpm, uint8_t IntsPerRotation = 2, uint8_t MinMeasurements = 2>
class RpmFromInterruptSource final : public IsrBaseHandler<typename InterruptSource::interrupt_type> {
    RpmFromInterruptSource() = delete;
public:
    typedef typename MCUTimer::mcu_type   mcu_type;
    typedef          MCUTimer             mcu_timer_type;
    typedef typename MCUTimer::value_type value_type;
    
    static_assert(sizeof (value_type) >= 2, "timer at least 16bit");
    static constexpr uint32_t minPeriod = 50;
    
    static constexpr auto prescaler = calculateRpm<MCUTimer>(MaxRpm, MinRpm, minPeriod);
    static_assert(prescaler > 0);
    
    static constexpr std::hertz fTimer = Config::fMcu / (uint32_t)prescaler;
    
    static void init() {
        MCUTimer::template prescale<prescaler>();
        MCUTimer::mode(AVR::TimerMode::Normal);
        InterruptSource::init();
    }
    
    inline static value_type period() {
        if constexpr(mcu_type::template is_atomic<value_type>()) {
            return mActualPeriod;
        }
        else {
            Scoped<DisbaleInterrupt<RestoreState>> di;
            return mActualPeriod;
        }
    }

    inline static value_type filteredPeriod() {
        if (mMeasurements > MinMeasurements) {
            if (mActualPeriod >= minPeriod) {
                return period();
            }
        }
        return 0;
    }
    
    inline static void reset() {
        mMeasurements = 0;
    }
    
    inline static std::hertz frequency() {
        if (filteredPeriod() > 0) {
            return fTimer / (uint32_t)filteredPeriod();
        }
        return {0};
    }

    inline static std::RPM rpm() {
        if (frequency().value > 0) {
            return std::RPM{frequency()};
        }
        return std::RPM{0};
    }
    
    inline static void isr() {
        if (++mIntCount == IntsPerRotation) {
            mIntCount = 0;
            mActualPeriod = (MCUTimer::counter() - mTimerStartValue + std::numeric_limits<value_type>::module()) % std::numeric_limits<value_type>::module();
            mTimerStartValue = MCUTimer::counter();
            ++mMeasurements;
        }
    }
private:
    inline static volatile value_type mTimerStartValue = 0;
    inline static volatile value_type mActualPeriod = 0;
    inline static volatile uint8_t mIntCount = 0;
    inline static volatile uint_bounded<uint8_t> mMeasurements{0};
};

template<typename MCUTimer, const std::RPM& MaxRpm, const std::RPM& MinRpm, uint8_t MinMeasurements = 2>
class RpmWithIcp final {
    RpmWithIcp() = delete;
public:
    typedef typename MCUTimer::mcu_type   mcu_type;
    typedef          MCUTimer             mcu_timer_type;
    typedef typename MCUTimer::value_type value_type;
    
    static_assert(sizeof (value_type) >= 2, "timer at least 16bit");
    static constexpr uint32_t minPeriod = 50;
    
    static constexpr auto prescaler = calculateRpm<MCUTimer>(MaxRpm, MinRpm, minPeriod);
    static_assert(prescaler > 0);
    
    static constexpr std::hertz fTimer = Config::fMcu / (uint32_t)prescaler;
    
    static void init() {
        MCUTimer::template prescale<prescaler>();
        MCUTimer::mode(AVR::TimerMode::IcpNoInt);
    }
    
    inline static value_type period() {
        if constexpr(mcu_type::template is_atomic<value_type>()) {
            return mActualPeriod;
        }
        else {
            Scoped<DisbaleInterrupt<RestoreState>> di;
            return mActualPeriod;
        }
    }

    inline static value_type filteredPeriod() {
        if (mMeasurements >= MinMeasurements) {
            if (mActualPeriod >= minPeriod) {
                return period();
            }
        }
        return 0;
    }
    
    inline static void reset() {
        mMeasurements = 0;
    }
    
    inline static void check() {
        static uint8_t lastNumberOfMeasurements = 0;
        if (mMeasurements.isTop()) {
            mMeasurements = MinMeasurements;
        }
        else {
            if (mMeasurements == lastNumberOfMeasurements) {
                reset();
            }
        }
        lastNumberOfMeasurements = mMeasurements;
    }
    
    inline static std::hertz frequency() {
        if (filteredPeriod() > 0) {
            return fTimer / (uint32_t)filteredPeriod();
        }
        return {0};
    }

    inline static std::RPM rpm() {
        if (frequency().value > 0) {
            return std::RPM{frequency()};
        }
        return std::RPM{0};
    }

    inline static void periodic() {
        MCUTimer::template periodic<MCUTimer::flags_type::icf>([](){
            mActualPeriod =(MCUTimer::icr() - mTimerStartValue + std::numeric_limits<value_type>::module()) % std::numeric_limits<value_type>::module();
            mTimerStartValue = MCUTimer::icr();
            ++mMeasurements;
        });
    }
    
//    inline static void isr() {
//            mActualPeriod = (MCUTimer::counter() - mTimerStartValue + std::numeric_limits<value_type>::module()) % std::numeric_limits<value_type>::module();
//            mTimerStartValue = MCUTimer::counter();
//            ++mMeasurements;
//    }
private:
    inline static volatile value_type mTimerStartValue = 0;
    inline static volatile value_type mActualPeriod = 0;
    inline static volatile uint_bounded<uint8_t> mMeasurements{0};
};

