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
#include "units/physical.h"


template<typename PinChange, typename MCUTimer>
class PpmDecoder final : public IsrBaseHandler<AVR::ISR::PcInt<PinChange::pcInterruptNumber>>{
    template<typename... II> friend class IsrRegistrar;
public:
    PpmDecoder() = delete;

    typedef MCUTimer mcu_timer_type;
    typedef typename MCUTimer::value_type  value_type;
    typedef typename MCUTimer::mcu_type mcu_type;
    typedef typename PinChange::pinset_type pinset_type;
    
    inline static constexpr auto mcuTimer = MCUTimer::mcuTimer;
    inline static constexpr uint16_t prescaler = AVR::Util::calculatePpmInParameter<MCUTimer, value_type>();
    inline static constexpr std::hertz timerFrequency = Config::fMcu / (uint32_t)prescaler;
    inline static constexpr value_type ppmMin = 1_ms * timerFrequency;
    inline static constexpr value_type ppmMax = 2_ms * timerFrequency;

    static_assert(ppmMin >= 10, "wrong prescaler");
    static_assert(ppmMax <= std::numeric_limits<value_type>::max(), "wrong prescaler");

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
            return period[index];
        }
        else {
            Scoped<DisbaleInterrupt<RestoreState>> di;
            return period[index];
        }
    }
    template<uint8_t Index>
    static inline value_type value() {
        if constexpr(mcu_type::template is_atomic<value_type>()) {
            return period[Index];
        }
        else {
            Scoped<DisbaleInterrupt<>> di;
            return period[Index];
        }
    }

private:
    template<uint8_t N>
    static inline void check(std::byte last, std::byte value) {
        if (std::any((last ^ value) & pinset_type::pinMasks[N])) {
            if (std::none(value & pinset_type::pinMasks[N])) { // high -> low
                period[N] = (*mcuTimer()->tcnt + std::numeric_limits<value_type>::module() - timerStartValue[N]) % std::numeric_limits<value_type>::module();
            }
            else { // low ->high
                timerStartValue[N] = *mcuTimer()->tcnt;
            }
        }
        if constexpr(N > 0) {
            check<N - 1>(last, value);
        }
    }

    static inline void isr() {
        static std::byte last_value{0};
        std::byte v = pinset_type::read();
        check<pinset_type::size - 1>(last_value, v);
        last_value = v;
    }
    inline static volatile std::array<value_type, pinset_type::size> period;
    inline static volatile std::array<value_type, pinset_type::size> timerStartValue;
};
