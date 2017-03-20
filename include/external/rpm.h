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

#include <stdint.h>
#include "std/limits.h"
#include "std/types.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/mcutimer.h"
#include "util/bits.h"
#include "util/disable.h"
#include "units/physical.h"

// 1Hz messbar mit 16-bit Zähler
// Ftimer >= 65535Hz
template<typename MCUTimer>
constexpr uint16_t calculateRpm() {
    using pBits = typename MCUTimer::mcu_timer_type::template PrescalerBits<MCUTimer::number>;
    auto p = AVR::Util::prescalerValues(pBits::values);
    auto sortedPRow = ::Util::sort(p, std::greater<typename AVR::PrescalerPair<typename MCUTimer::tccrb_type>::scale_type>()); // absteigend

    for(const auto& p : sortedPRow) {
        if (p > 0) {
            const std::hertz f = Config::fMcu / (uint32_t)p;
            if (f >= 65535_Hz) {
                return p;
            }
        }
    }
    return 0;
}

// todo: Drehzahlbereich festlegen -> Timerprescaler
template<typename InterruptSource, typename MCUTimer, const std::RPM& MaxRpm, uint8_t IntsPerRotation = 2, uint8_t MinMeasurements = 2>
class RpmFromInterruptSource final : public IsrBaseHandler<typename InterruptSource::interrupt_type> {
    RpmFromInterruptSource() = delete;
public:
    typedef typename MCUTimer::mcu_type   mcu_type;
    typedef          MCUTimer             mcu_timer_type;
    typedef typename MCUTimer::value_type value_type;
    
    static_assert(sizeof (value_type) >= 2, "timer at least 16bit");
    static constexpr auto prescaler = calculateRpm<MCUTimer>();
    static constexpr std::hertz fTimer = Config::fMcu / (uint32_t)prescaler;
    
    static constexpr auto minPeriod = (fTimer.value * 60) / MaxRpm.value();
    
    static void init() {
        MCUTimer::template prescale<prescaler>();
        MCUTimer::mode(AVR::TimerMode::Normal);
        InterruptSource::init();
    }
    
    static value_type period() {
        if constexpr(mcu_type::template is_atomic<value_type>()) {
            return mActualPeriod();
        }
        else {
            Scoped<DisbaleInterrupt> di;
            return mActualPeriod;
        }
    }

    static value_type filteredPeriod() {
        if (mMeasurements > MinMeasurements) {
            if (mActualPeriod >= minPeriod) {
                return mActualPeriod;
            }
        }
        return 0;
    }
    
    static void reset() {
        mMeasurements = 0;
    }
    
    static std::hertz frequency() {
        if (filteredPeriod() > 0) {
            return mcu_timer_type::frequency() / (uint32_t)filteredPeriod();
        }
        return {0};
    }

    static std::RPM rpm() {
        if (frequency().value > 0) {
            return std::RPM{frequency()};
        }
        return std::RPM{};
    }
    
    static void isr() {
        if (++mIntCount == IntsPerRotation) {
            mActualPeriod = (MCUTimer::counter() - mTimerStartValue + std::numeric_limits<value_type>::module()) % std::numeric_limits<value_type>::module();
            mTimerStartValue = MCUTimer::counter();
            mIntCount = 0;
            ++mMeasurements;
        }
    }
private:
    inline static volatile value_type mTimerStartValue = 0;
    inline static volatile value_type mActualPeriod = 0;
    inline static volatile uint8_t mIntCount = 0;
    inline static volatile uint_bounded<uint8_t> mMeasurements{0};
};

