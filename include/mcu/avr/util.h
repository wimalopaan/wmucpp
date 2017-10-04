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

#include "config.h"
#include "avr8defs.h"
#include "std/limits.h"
#include "units/physical.h"
#include "util/algorithm.h"

namespace AVR {
    namespace Util {
        
        template<typename T>
        struct TimerSetupData final {
            const uint16_t prescaler = 0;
            const T ocr = 0;
            const std::hertz f{0};
            const bool isExact = false;
            explicit constexpr operator bool() const {
                return (prescaler > 0) && (ocr > 0);
            }
        };
        
        template<typename T, uint16_t N>
        constexpr std::array<typename AVR::PrescalerPair<T>::scale_type, N> prescalerValues(const std::array<AVR::PrescalerPair<T>, N>& a) {
            std::array<typename AVR::PrescalerPair<T>::scale_type, N> values;
            for(uint8_t i = 0; i < N; ++i) {
                values[i] = a[i].scale;
            }
            return values;
        }
        
        template<uint16_t Prescale, typename T, uint16_t N>
        constexpr typename AVR::PrescalerPair<T>::bits_type bitsFrom(const std::array<AVR::PrescalerPair<T>, N>& a) {
            for(const auto pair: a) {
                if (pair.scale == Prescale) {
                    return pair.bits;
                }
            }
            return static_cast<typename AVR::PrescalerPair<T>::bits_type>(0);
        }
        
        template<typename T, uint16_t N>
        constexpr uint16_t bitsToPrescale(T bits, const std::array<AVR::PrescalerPair<T>, N>& a) {
            for(const auto& pair : a) {
                if (bits == pair.bits) {
                    return pair.scale;
                }
            }
            return 0;
        }
        
        template<typename MCUTimer>
        constexpr TimerSetupData<typename MCUTimer::value_type> calculate(const std::hertz& ftimer) {
            //    static_assert(MCUTimer::hasOcrA || MCUTimer::hasOcrB, "need ocra or ocrb");
            
            using pBits = typename MCUTimer::mcu_timer_type::template PrescalerBits<MCUTimer::number>;
            auto p = prescalerValues(pBits::values);
            
            for(const auto& p : ::Util::sort(p)) { // aufsteigend
                if (p > 0) {
                    const auto tv = (Config::fMcu / ftimer) / p;
                    if ((tv > 0) && (tv < std::numeric_limits<typename MCUTimer::value_type>::max())) {
                        const bool exact = ((Config::fMcu.value / p) % tv) == 0;
                        return {p, static_cast<typename MCUTimer::value_type>(tv), Config::fMcu / tv / uint32_t(p), exact};
                    }
                }
            }
            return {};
        }
        
        template<typename MCUTimer>
        constexpr uint16_t prescalerForAbove(const std::hertz& ftimer) {
            using pBits = typename MCUTimer::mcu_timer_type::template PrescalerBits<MCUTimer::number>;
            auto p = prescalerValues(pBits::values);
            for(const auto& p : ::Util::sort(p, std::less<uint16_t>())) {
                if (p > 0) {
                    auto f = Config::fMcu / p;
                    if (f >= ftimer) {
                        return p;
                    }
                }
            }
            return 0;
        }
        
        template<typename MCUTimer, typename T>
        constexpr uint16_t calculatePpmInParameter() {
            using pBits = typename MCUTimer::mcu_timer_type::template PrescalerBits<MCUTimer::number>;
            auto p = AVR::Util::prescalerValues(pBits::values);
            
            for(const auto& p : ::Util::sort(p)) {
                if (p > 0) {
                    const std::hertz f = Config::fMcu / p;
                    const uint16_t ppmMin = 1_ms * f;
                    const uint16_t ppmMax = 2_ms * f;
                    if ((ppmMax < std::numeric_limits<T>::max()) && (ppmMin > 10)) {
                        return p;
                    }
                }
            }
            return 0;
        }
        
        template<typename MCUTimer, typename T>
        constexpr uint16_t calculatePpmOutParameter() {
            using pBits = typename MCUTimer::mcu_timer_type::template PrescalerBits<MCUTimer::number>;
            auto p = AVR::Util::prescalerValues(pBits::values);
            
            for(const auto& p : ::Util::sort(p)) {
                if (p > 0) {
                    const std::hertz f = Config::fMcu / p;
                    const uint32_t ppmMin = 1_ms * f;
                    const uint32_t ppmMax = 20_ms * f;
                    if ((ppmMax < std::numeric_limits<T>::max()) && (ppmMin > 1)) {
                        return p;
                    }
                }
            }
            return 0;
        }
        
        template<typename MCUTimer>
        constexpr TimerSetupData<typename MCUTimer::value_type> caculateForExactFrequencyAbove(const std::hertz& f) {
            using pBits = typename MCUTimer::mcu_timer_type::template PrescalerBits<MCUTimer::number>;
            auto p = AVR::Util::prescalerValues(pBits::values);
            
            for(const auto& p : ::Util::sort(p, std::greater<uint16_t>())) { // absteigend
                if (p > 0) {
                    const auto timerStartValue = (Config::fMcu / f) / p;
                    const decltype(timerStartValue) tmax = std::numeric_limits<typename MCUTimer::value_type>::max();
                    for(auto tx = std::min(timerStartValue, tmax); tx > 0; --tx) {
                        const auto fx = (Config::fMcu / p) / tx;
                        if (fx <= (2 * f)) {
                            uint32_t uf = Config::fMcu.value;
                            const auto rx = uf % (p * tx); 
                            if (rx == 0) {
                                return {p, static_cast<typename MCUTimer::value_type>(tx), fx, true};
                            }
                        }
                    }
                }
            }
            return{0};
        }
        
        
        
    }
}
