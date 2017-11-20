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

#include "mcu/ports.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/util.h"
#include "mcu/avr/ppmbase.h"
#include "units/percent.h"
#include "util/disable.h"

// todo: PinSet einsetzen
template<typename Timer, typename... Pins>
class SoftPPM final {
    SoftPPM() = delete;
    
public:
    static_assert(std::is_same<typename Timer::value_type, uint16_t>::value, "must use 16bit timer");
    
    // constexpr umrechnen der Pins... in ein constexpr array mit Masken und ... s.a PinSet
    static constexpr const uint8_t numberOfChannels = sizeof...(Pins);
    static constexpr auto mcuTimer = Timer::mcuTimer;
    static constexpr auto mcuInterrupts = Timer::mcuInterrupts;
    
    using parameter = PPMParameter<Timer>;
    
    static void timerInit() {
        Timer::template prescale<parameter::prescaler>();
        *mcuTimer()->ocra = parameter::ocFrame;
        mcuTimer()->tccrb.template add<Timer::tccrb_type::wgm2>();
        mcuInterrupts()->tifr.template add<Timer::flags_type::ocfa | Timer::flags_type::ocfb>();
        mcuInterrupts()->timsk.template add<Timer::mask_type::ociea>();
    }
    
    static void init() {
        (Pins::low(), ...);
        (Pins::template dir<AVR::Output>(), ...);
        timerInit();
        constexpr auto medium = (parameter::ocMax + parameter::ocMin) / 2;
        std::iota(std::begin(ocrbValues), std::end(ocrbValues), medium, medium);
    }
    
    static void ppm(const std::percent& width, uint8_t channel) {
        assert(channel < numberOfChannels);
        uint16_t ocr = std::expand(width, parameter::ocMin, parameter::ocMax);
        uint16_t start = 0;
        for(uint8_t i = 0; i < channel; ++i) {
            start += ocrbValues[i];
        }
        uint16_t end = start + ocr;
        int16_t diff = ocr - ocrbValues[channel];
        {
            Scoped<DisbaleInterrupt<>> di;
            ocrbValues[channel] = end;
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
    
    // todo: etwas eleganter !!! Auf die Klasse verzichten, weil keine partielle Spezialisierung mehr erforderlich wegen constexpr
    template<uint8_t N, typename P, typename... PP>
    struct OffN {
        static void check(uint8_t i) {
            if ((numberOfChannels - 1 - i) == (N - 1))  {
                P::low();
            }
            else if constexpr((N - 1) > 0) {
                OffN<N - 1, PP..., void>::check(i);
            }
        }
    };
    
    template<uint8_t N, typename P, typename... PP>
    struct OnN {
        static void check(uint8_t i) {
            if ((numberOfChannels - 1 - i) == (N - 1))  {
                P::high();
            }
            else if constexpr((N - 1) > 0) {
                OnN<N - 1, PP..., void>::check(i);
            }
        }
    };
    
    struct OCAHandler : public IsrBaseHandler<typename AVR::ISR::Timer<Timer::number>::CompareA> {
        static void isr() {
            actual = 0;
            *mcuTimer()->ocrb = ocrbValues[0];
            mcuInterrupts()->timsk.template add<Timer::mask_type::ocieb>();
            First<Pins...>::high();
        }
    };
    struct OCBHandler : public IsrBaseHandler<typename AVR::ISR::Timer<Timer::number>::CompareB> {
        static void isr() {
            OffN<numberOfChannels, Pins...>::check(actual);
            actual = (actual + 1) % numberOfChannels;
            if (actual != 0) {
                OnN<numberOfChannels, Pins...>::check(actual);
                *mcuTimer()->ocrb = ocrbValues[actual];
            }
            else {
                if constexpr(numberOfChannels > 1) {
                    mcuInterrupts()->timsk.template clear<Timer::mask_type::ocieb>();
                }
            }
        }
    };
private:
    inline static volatile uint8_t actual = 0;
    inline static volatile uint16_t ocrbValues[numberOfChannels] = {};
};



