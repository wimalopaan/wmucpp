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

#include "config.h"
#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/util.h"
#include "util/disable.h"
#include "util/types.h"
#include "units/physical.h"

// fixme: flag-Regsiter

template<typename PinChange, typename MCUTimer, auto Channels = 8>
class SumO final : public IsrBaseHandler<AVR::ISR::PcInt<PinChange::pcInterruptNumber>>{
    template<typename... II> friend class IsrRegistrar;
    
public:
    SumO() = delete;

    typedef MCUTimer mcu_timer_type;
    typedef typename MCUTimer::value_type  value_type;
    typedef typename MCUTimer::mcu_type mcu_type;
    typedef typename PinChange::pinset_type pinset_type;
    static_assert(pinset_type::size == 1, "use only one pin in pinset");
    
    using pin = Meta::front<typename pinset_type::pinlist>;
    
    inline static constexpr auto mcuTimer = MCUTimer::mcuTimer;
    inline static constexpr uint16_t prescaler = AVR::Util::calculatePpmOutParameter<MCUTimer, value_type>(); // !!!
    inline static constexpr std::hertz timerFrequency = Config::fMcu / (uint32_t)prescaler;
    inline static constexpr value_type ppmMin = 1_ms * timerFrequency;
    inline static constexpr value_type ppmMax = 2_ms * timerFrequency;
    inline static constexpr value_type ppmCycle = 20_ms * timerFrequency;
    inline static constexpr value_type ppmSync = ppmCycle - (Channels * ppmMax);

    static_assert(ppmMin >= 10, "wrong prescaler");
    static_assert(ppmMax <= std::numeric_limits<value_type>::max(), "wrong prescaler");
    static_assert(ppmMax > ppmMin, "wrong prescaler");
    static_assert(ppmCycle <= std::numeric_limits<value_type>::max(), "wrong prescaler");
    static_assert(ppmCycle > ppmMax, "wrong prescaler");
    static_assert(ppmSync > ppmMax, "wrong prescaler");

    inline static constexpr value_type ppmMid = (ppmMax + ppmMin) / 2;
    inline static constexpr value_type ppmDelta = (ppmMax - ppmMin) / 5;
    inline static constexpr value_type ppmMidLow = ppmMid - ppmDelta;
    inline static constexpr value_type ppmMidHigh = ppmMid + ppmDelta;
    inline static constexpr value_type ppmMinHigh = ppmMin + ppmDelta;
    inline static constexpr value_type ppmMaxLow = ppmMax - ppmDelta;

    static inline void init() {
        PinChange::init();
        MCUTimer::template prescale<prescaler>();
        MCUTimer::mode(AVR::TimerMode::Normal);
        MCUTimer::start();
    }
    
    static inline value_type value(uint8_t index) {
        if constexpr(mcu_type::template is_atomic<value_type>()) {
            return channels[index];
        }
        else {
            Scoped<DisbaleInterrupt<RestoreState>> di;
            return channels[index];
        }
    }
    template<uint8_t Index>
    static inline value_type value() {
        if constexpr(mcu_type::template is_atomic<value_type>()) {
            return channels[Index];
        }
        else {
            Scoped<DisbaleInterrupt<>> di;
            return channels[Index];
        }
    }

private:
    template<typename T>
    static inline T cdiff(T actual, T start) {
        if (actual >= start) {
            return actual - start;
        }
        else {
            return (std::numeric_limits<T>::module() - start) + actual;
        }
    }
    static inline void isr() {
        if (!wasLow && !pin::isHigh()){ // falling
            wasLow = true;
            value_type actual = *mcuTimer()->tcnt;
            value_type highPeriod = cdiff(actual, mTimerStartValueRising);
            value_type low2lowPeriod = cdiff(actual, mTimerStartValueFalling);
            mTimerStartValueFalling = actual;
            if (highPeriod >= ppmSync) {
                channel = 0;
            }
            else {
                if (channel < Channels) {
                    channels[channel] = low2lowPeriod;
                    ++channel;
                }
                else {
                    channel = 0;
                }
            }
        }
        else if (wasLow && pin::isHigh()) { // rising
            wasLow = false;
            mTimerStartValueRising = *mcuTimer()->tcnt;
        } 
    }
    
    inline static volatile std::array<value_type, Channels> channels;
    inline static uint8_t channel = 0;
    inline static value_type mTimerStartValueRising = 0;
    inline static value_type mTimerStartValueFalling = 0;
    
    inline static bool wasLow = false;
};
