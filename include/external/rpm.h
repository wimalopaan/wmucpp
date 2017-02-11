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
            return actualPeriod();
        }
        else {
            Scoped<DisbaleInterrupt> di;
            return actualPeriod();
        }
    }

    static value_type filteredPeriod() {
        if (measurements() > MinMeasurements) {
            if (actualPeriod() >= minPeriod) {
                return actualPeriod();
            }
        }
        return 0;
    }
    
    static void reset() {
        measurements()= 0;
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
        if (++intCount() == IntsPerRotation) {
            actualPeriod() = (MCUTimer::counter() - timerStartValue() + std::numeric_limits<value_type>::module()) % std::numeric_limits<value_type>::module();
            timerStartValue() = MCUTimer::counter();
            intCount() = 0;
            ++measurements();
        }
    }
private:
    static auto& timerStartValue() {
        static volatile value_type sTimerStartValue = 0;
        return sTimerStartValue;
    }
    static auto& actualPeriod() {
        static volatile value_type sPeriod = 0;
        return sPeriod;
    }
    static auto& intCount() {
        static volatile uint8_t sIntCount = 0;
        return sIntCount;
    }
    static auto& measurements() {
        static volatile uint_bounded<uint8_t> sMeasurements{0};
        return sMeasurements;
    }
};

