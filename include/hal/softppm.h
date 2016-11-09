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

#include "mcu/ports.h"
#include "units/percent.h"
#include "util/disable.h"

template<typename MCUTimer, typename T>
constexpr uint16_t calculatePwm() {
    using pRow = typename MCUTimer::mcu_timer_type::template PrescalerRow<MCUTimer::number>;
    for(const auto& p : pRow::values) {
        const std::hertz f = Config::fMcu / p;
        const uint32_t ppmMin = 1_ms * f;
        const uint32_t ppmMax = 20_ms * f;
        if ((ppmMax < std::numerical_limits<T>::max()) && (ppmMin > 1)) {
            return p;
        }
    }
    return 0;
}

template<typename MCUTimer>
class SoftPWMBase {
public:
    SoftPWMBase() = delete;
    static constexpr auto mcuTimer = MCUTimer::mcuTimer;
    static constexpr auto mcuInterrupts = MCUTimer::mcuInterrupts;
    static constexpr uint16_t prescaler = calculatePwm<MCUTimer, uint16_t>();
    static constexpr std::hertz timerFrequency = Config::fMcu / prescaler;
    static constexpr uint16_t ocMin = 1_ms * timerFrequency;
    static constexpr uint16_t ocMax = 2_ms * timerFrequency;
    static constexpr uint16_t ocDelta = ocMax - ocMin;
    static constexpr uint16_t ocFrame = 20_ms * timerFrequency;

    static void timerInit() {
        mcuTimer->ocra = ocFrame;
        mcuTimer->tccrb |= _BV(WGM12);
        mcuInterrupts->tifr  |= _BV(OCF1A) | _BV(OCF1B);
        mcuInterrupts->timsk |= _BV(OCIE0A) | _BV(OCIE0B);
        MCUTimer::template prescale<prescaler>();
    }
};
template<typename MCUTimer>
constexpr std::hertz SoftPWMBase<MCUTimer>::timerFrequency;

template<typename MCUTimer, typename... Pins>
class SoftPWM : public SoftPWMBase<MCUTimer> {
public:
    SoftPWM() = delete;
    static constexpr const uint8_t numberOfChannels = sizeof...(Pins);

    using SoftPWMBase<MCUTimer>::timerInit;
    using SoftPWMBase<MCUTimer>::ocMin;
    using SoftPWMBase<MCUTimer>::ocMax;
    using SoftPWMBase<MCUTimer>::mcuTimer;

    static void init() {
        (Pins::low(), ...);
        (Pins::template dir<AVR::Output>(), ...);
        timerInit();
        std::iota(std::begin(ocrbValues), std::end(ocrbValues), (ocMax + ocMin) / 2);
    }

    static void pwm(const std::percent& width, uint8_t channel) {
        assert(channel < numberOfChannels);
        uint16_t ocr = std::expand(width, ocMin, ocMax);
        uint16_t diff = ocr - ocrbValues[channel];
        {
            // todo: check
            Scoped<DisbaleInterrupt> di;
            ocrbValues[channel] = ocr;
            for(uint8_t i = channel + 1; i < numberOfChannels; ++i) {
                ocrbValues[i] += diff;
            }
        }
    }

    template<typename P, typename...PP>
    struct First {
      static void high() {
            P::high();
        }
      static void low() {
            P::low();
        }
    };
    template<uint8_t N, typename P, typename... PP>
    struct OffOnN {
        static void check(uint8_t i) {
            if (i == (N - 1))  {
                P::high();
            }
            else {
                P::low();
            }
            OffOnN<N - 1, PP..., void>::check(i);
        }
    };
    template<typename... PP>
    struct OffOnN<0, void, PP...> {
        static void check(uint8_t) {}
    };
    static void isrA() { // CTC
        actual = 0;
        mcuTimer->ocrb = ocrbValues[0];
        First<Pins...>::high();
    }
    static void isrB() {
        OffOnN<numberOfChannels, Pins...>::check(actual);
        actual = (actual + 1) & numberOfChannels;
        mcuTimer->ocrb = ocrbValues[actual];
    }
private:
    static uint8_t actual;
    static uint16_t ocrbValues[numberOfChannels];
};
template<typename MCUTimer, typename... Pins>
uint8_t SoftPWM<MCUTimer, Pins...>::actual = 0;

template<typename MCUTimer, typename... Pins>
uint16_t SoftPWM<MCUTimer, Pins...>::ocrbValues[SoftPWM<MCUTimer, Pins...>::numberOfChannels] = {};



