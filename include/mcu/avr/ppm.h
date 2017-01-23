/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

// todo: in Util::AVR
template<typename MCUTimer, typename T>
constexpr uint16_t calculatePpm() {
    using pBits = typename MCUTimer::mcu_timer_type::template PrescalerBits<MCUTimer::number>;
    auto p = AVR::Util::prescalerValues(pBits::values);
    auto sortedPRow = ::Util::sort(p);

    for(const auto& p : sortedPRow) {
        const std::hertz f = Config::fMcu / (uint32_t)p;
        const uint16_t ppmMin = 1_ms * f;
        const uint16_t ppmMax = 2_ms * f;
        if ((ppmMax < std::numeric_limits<T>::max()) && (ppmMin > 10)) {
            return p;
        }
    }
    return 0;
}

template<typename PinChange, typename MCUTimer>
class PpmDecoder final : public IsrBaseHandler<AVR::ISR::PcInt<PinChange::pcGroupNumber>>{
    template<typename... II> friend class IsrRegistrar;
    friend void ::PCINT0_vect();
    friend void ::PCINT1_vect();
    friend void ::PCINT2_vect();
    friend void ::PCINT3_vect();
public:
    PpmDecoder() = delete;

    typedef MCUTimer mcu_timer_type;
    typedef typename MCUTimer::value_type  value_type;
    typedef typename MCUTimer::mcu_type mcu_type;
    typedef typename PinChange::pinset_type pinset_type;
    
    static constexpr auto mcuTimer = MCUTimer::mcuTimer;
    static constexpr uint16_t prescaler = calculatePpm<MCUTimer, value_type>();
    static constexpr std::hertz timerFrequency = Config::fMcu / (uint32_t)prescaler;
    static constexpr value_type ppmMin = 1_ms * timerFrequency;
    static constexpr value_type ppmMax = 2_ms * timerFrequency;

    static_assert(ppmMin >= 10, "wrong prescaler");
    static_assert(ppmMax <= 255, "wrong prescaler");

    static constexpr value_type ppmMid = (ppmMax + ppmMin) / 2;
    static constexpr value_type ppmDelta = (ppmMax - ppmMin) / 5;
    static constexpr value_type ppmMidLow = ppmMid - ppmDelta;
    static constexpr value_type ppmMidHigh = ppmMid + ppmDelta;
    static constexpr value_type ppmMinHigh = ppmMin + ppmDelta;
    static constexpr value_type ppmMaxLow = ppmMax - ppmDelta;

    static void init() {
        MCUTimer::template prescale<prescaler>();
        MCUTimer::mode(AVR::TimerMode::Normal);
        MCUTimer::start();
    }
    
    static value_type value(uint8_t index) {
        if constexpr(mcu_type::template is_atomic<value_type>()) {
            return period[index];
        }
        else {
            Scoped<DisbaleInterrupt> di;
            return period[index];
        }
    }
    template<uint8_t Index>
    static value_type value() {
        if constexpr(mcu_type::template is_atomic<value_type>()) {
            return period[Index];
        }
        else {
            Scoped<DisbaleInterrupt> di;
            return period[Index];
        }
    }

private:
    template<uint8_t N>
    static inline void check(uint8_t last, uint8_t value) {
        if ((last ^ value) & pinset_type::pinMasks[N]) {
            if (!(value & pinset_type::pinMasks[N])) { // high -> low
                period[N] = (mcuTimer()->tcnt + std::numeric_limits<value_type>::module() - timerStartValue[N]) % std::numeric_limits<value_type>::module();
            }
            else { // low ->high
                timerStartValue[N] = mcuTimer()->tcnt;
            }
        }
        if constexpr(N > 0) {
            check<N - 1>(last, value);
        }
    }

    static void isr() {
        static uint8_t last_value = 0;
        uint8_t v = pinset_type::read();
        check<pinset_type::size - 1>(last_value, v);
        last_value = v;
    }
    static volatile std::array<value_type, pinset_type::size> period;
    static volatile std::array<value_type, pinset_type::size> timerStartValue;
};

template<typename PinChange, typename MCUTimer>
volatile std::array<typename PpmDecoder<PinChange, MCUTimer>::value_type, PpmDecoder<PinChange, MCUTimer>::pinset_type::size> PpmDecoder<PinChange, MCUTimer>::period;

template<typename PinChange, typename MCUTimer>
volatile std::array<typename PpmDecoder<PinChange, MCUTimer>::value_type, PpmDecoder<PinChange, MCUTimer>::pinset_type::size> PpmDecoder<PinChange, MCUTimer>::timerStartValue;

template<typename PinChange, typename MCUTimer>
constexpr std::hertz PpmDecoder<PinChange, MCUTimer>::timerFrequency;
