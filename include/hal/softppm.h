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

template<typename MCUTimer, typename... Pins>
class SoftPPM final : public PPMBase<MCUTimer> {
    SoftPPM() = delete;

public:
    static_assert(std::is_same<typename MCUTimer::value_type, uint16_t>::value, "must use 16bit timer");
    
    static constexpr const uint8_t numberOfChannels = sizeof...(Pins);

    using PPMBase<MCUTimer>::mcuInterrupts;
    using PPMBase<MCUTimer>::ocMin;
    using PPMBase<MCUTimer>::ocMax;
    using PPMBase<MCUTimer>::ocFrame;
    using PPMBase<MCUTimer>::mcuTimer;
    using PPMBase<MCUTimer>::prescaler;

    static void timerInit() {
        MCUTimer::template prescale<prescaler>();
        *mcuTimer()->ocra = ocFrame;
        mcuTimer()->tccrb.template add<MCUTimer::tccrb_type::wgm2>();
        mcuInterrupts()->tifr.template add<MCUTimer::flags_type::ocfa | MCUTimer::flags_type::ocfb>();
        mcuInterrupts()->timsk.template add<MCUTimer::mask_type::ociea>();
    }
    
    static void init() {
        (Pins::low(), ...);
        (Pins::template dir<AVR::Output>(), ...);
        timerInit();
        std::iota(std::begin(ocrbValues), std::end(ocrbValues), (ocMax + ocMin) / 2, (ocMax + ocMin) / 2);
    }

    static void ppm(const std::percent& width, uint8_t channel) {
        assert(channel < numberOfChannels);
        uint16_t ocr = std::expand(width, ocMin, ocMax);
        {
            Scoped<DisbaleInterrupt> di;
            uint16_t start = 0;
            for(uint8_t i = 0; i < channel; ++i) {
                start += ocrbValues[i];
            }
            uint16_t end = start + ocr;
            int16_t diff = ocr - ocrbValues[channel];
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

    struct OCAHandler : public IsrBaseHandler<typename AVR::ISR::Timer<MCUTimer::number>::CompareA> {
        static void isr() {
            actual = 0;
            *mcuTimer()->ocrb = ocrbValues[0];
            mcuInterrupts()->timsk.template add<MCUTimer::mask_type::ocieb>();
            First<Pins...>::high();
        }
    };
    struct OCBHandler : public IsrBaseHandler<typename AVR::ISR::Timer<MCUTimer::number>::CompareB> {
        static void isr() {
            OffN<numberOfChannels, Pins...>::check(actual);
            actual = (actual + 1) % numberOfChannels;
            if (actual != 0) {
                OnN<numberOfChannels, Pins...>::check(actual);
                *mcuTimer()->ocrb = ocrbValues[actual];
            }
            else {
                if constexpr(numberOfChannels > 1) {
                    mcuInterrupts()->timsk.template clear<MCUTimer::mask_type::ocieb>();
                }
            }
        }
    };
    static volatile uint8_t actual;
    static volatile uint16_t ocrbValues[numberOfChannels];
private:
};
template<typename MCUTimer, typename... Pins>
volatile uint8_t SoftPPM<MCUTimer, Pins...>::actual = 0;

template<typename MCUTimer, typename... Pins>
volatile uint16_t SoftPPM<MCUTimer, Pins...>::ocrbValues[SoftPPM<MCUTimer, Pins...>::numberOfChannels] = {};



