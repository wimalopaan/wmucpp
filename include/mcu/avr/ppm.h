/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include "units/physical.h"

// todo: generalisieren als PinChangeDecoder, auch fuer mehrere Pins je Port, die sich ändern


template<typename MCUTimer, typename T>
constexpr uint16_t calculatePpm() {
    using pRow = typename MCUTimer::mcu_timer_type::template PrescalerRow<MCUTimer::number>;
    for(const auto& p : pRow::values) {
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
class PpmDecoder final {
    friend void ::PCINT0_vect();
    friend void ::PCINT1_vect();
    friend void ::PCINT2_vect();
    friend void ::PCINT3_vect();
public:
    PpmDecoder() = delete;

    typedef typename PinChange::pin_type pin_type;
    typedef MCUTimer mcu_timer_type;
    typedef typename MCUTimer::mcu_type mcu_type;

    static constexpr auto mcuTimer = MCUTimer::mcuTimer;
    static constexpr uint16_t prescaler = calculatePpm<MCUTimer, uint8_t>();
    static constexpr std::hertz timerFrequency = Config::fMcu / (uint32_t)prescaler;
    static constexpr uint8_t ppmMin = 1_ms * timerFrequency;
    static constexpr uint8_t ppmMax = 2_ms * timerFrequency;

    static_assert(ppmMin >= 10, "wrong prescaler");
    static_assert(ppmMax <= 255, "wrong prescaler");

    static constexpr uint8_t ppmMid = (ppmMax + ppmMin) / 2;
    static constexpr uint8_t ppmDelta = (ppmMax - ppmMin) / 5;
    static constexpr uint8_t ppmMidLow = ppmMid - ppmDelta;
    static constexpr uint8_t ppmMidHigh = ppmMid + ppmDelta;
    static constexpr uint8_t ppmMinHigh = ppmMin + ppmDelta;
    static constexpr uint8_t ppmMaxLow = ppmMax - ppmDelta;

    static void init() {
        MCUTimer::template prescale<prescaler>();
        MCUTimer::mode(AVR::TimerMode::Normal);
        MCUTimer::start();
    }

    static uint8_t value() {
        return period;
    }

private:
    static void isr0() {
    }
    static void isr1() {
    }
    static void isr2() {
        if (!pin_type::read()) { // high -> low
            uint8_t v = mcuTimer->tcnt;
            period = (v + 256 - timerStartValue) % 256;
        }
        else { // low -> high
            timerStartValue = mcuTimer->tcnt;
        }
    }
    static void isr3() {
    }
    static volatile uint8_t period;
    static volatile uint8_t timerStartValue;
};

template<typename PinChange, typename MCUTimer>
volatile uint8_t PpmDecoder<PinChange, MCUTimer>::period = 0;

template<typename PinChange, typename MCUTimer>
volatile uint8_t PpmDecoder<PinChange, MCUTimer>::timerStartValue = 0;

template<typename PinChange, typename MCUTimer>
constexpr std::hertz PpmDecoder<PinChange, MCUTimer>::timerFrequency;
