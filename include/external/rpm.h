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
#include "mcu/avr/isr.h"
#include "mcu/avr/mcutimer.h"
#include "util/bits.h"
#include "util/disable.h"
#include "units/physical.h"

struct RPM {
    explicit RPM(uint16_t v) : mValue{v} {}
    explicit RPM(const std::hertz& f) : mValue(f.value * 60) {}
    uint16_t mValue = 0;
};

template<typename Stream>
Stream& operator<<(Stream& o, const RPM& rpm) {
    if (!Config::disableCout) {
        return o << rpm.mValue << "RPM";
    }
    return o;
}

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
template<typename InterruptSource, typename MCUTimer, uint8_t IntsPerRotation = 2>
class RpmFromInterruptSource final : public IsrBaseHandler<typename InterruptSource::interrupt_type> {
    RpmFromInterruptSource() = delete;
public:
    typedef typename MCUTimer::mcu_type   mcu_type;
    typedef          MCUTimer             mcu_timer_type;
    typedef typename MCUTimer::value_type value_type;
    
    static_assert(sizeof (value_type) >= 2, "timer at least 16bit");
    
    static void init() {
        constexpr auto prescaler = calculateRpm<MCUTimer>();
        MCUTimer::template prescale<prescaler>();
        MCUTimer::mode(AVR::TimerMode::Normal);
        InterruptSource::init();
    }
    
    static value_type period() {
        if constexpr(mcu_type::template is_atomic<value_type>()) {
            return mPeriod;
        }
        else {
            Scoped<DisbaleInterrupt> di;
            return mPeriod;
        }
    }
    
    static void reset() {
        mPeriod = 0;
    }
    
    static std::hertz frequency() {
        if (mPeriod > 0) {
            return mcu_timer_type::frequency() / (uint32_t)mPeriod;
        }
        return {0};
    }

    static RPM rpm() {
        return RPM{frequency()};
    }
    
    static void isr() {
        ++mIntCount;
        if (mIntCount == IntsPerRotation) {
            mPeriod = (MCUTimer::counter() - mTimerStartValue + std::numeric_limits<value_type>::module()) % std::numeric_limits<value_type>::module();
            mTimerStartValue = MCUTimer::counter();
            mIntCount = 0;
        }
    }
private:
    static volatile value_type mTimerStartValue;
    static volatile value_type mPeriod;
    static volatile uint8_t mIntCount;
};

template<typename InterruptSource, typename MCUTimer, uint8_t IntsPerRotation>
volatile typename RpmFromInterruptSource<InterruptSource, MCUTimer, IntsPerRotation>::value_type RpmFromInterruptSource<InterruptSource, MCUTimer, IntsPerRotation>::mPeriod = 0;

template<typename InterruptSource, typename MCUTimer, uint8_t IntsPerRotation>
volatile typename RpmFromInterruptSource<InterruptSource, MCUTimer, IntsPerRotation>::value_type RpmFromInterruptSource<InterruptSource, MCUTimer, IntsPerRotation>::mTimerStartValue = 0;

template<typename InterruptSource, typename MCUTimer, uint8_t IntsPerRotation>
volatile uint8_t RpmFromInterruptSource<InterruptSource, MCUTimer, IntsPerRotation>::mIntCount = 0;

