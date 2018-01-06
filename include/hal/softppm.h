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
#include "util/meta.h"

//namespace detail {
//    template<typename List> 
//    struct visit {
//        template<typename T> struct Wrapper{
//            typedef T type;
//        };
//        using first = Meta::front<List>;
//        template<typename I, typename C>
//        inline static void at(I index, const C& callable) {
//            if (index == 0) {
//                callable(Wrapper<first>{});
//            }
//            else {
//                visit<Meta::rest<List>>::at(index - 1, callable);
//            }
//        }
//    };
//    template<> 
//    struct visit<Meta::List<>> {
//        template<typename I, typename C>
//        inline static void at(I, const C&) {}
//    };
//}

//template<typename List, typename I, typename C>
//inline void visitAt(I index, const C& callable) {
//    detail::visit<List>::at(index, callable);
//}

template<typename Timer, typename... Pins>
class SoftPPM final {
    SoftPPM() = delete;
    
public:
    static_assert(std::is_same<typename Timer::value_type, uint16_t>::value, "must use 16bit timer");
    
    static constexpr const uint8_t numberOfChannels = sizeof...(Pins);
    static constexpr auto mcuTimer = Timer::mcuTimer;
    static constexpr auto mcuInterrupts = Timer::mcuInterrupts;
    
    using parameter = PPMParameter<Timer>;

    typedef uint_ranged<uint16_t, parameter::ocMin, parameter::ocMax> ranged_type;
    
    using pin_list = Meta::List<Pins...>;
    using first_pin = Meta::front<pin_list>;
    
    inline static void timerInit() {
        Timer::template prescale<parameter::prescaler>();
        *mcuTimer()->ocra = parameter::ocFrame;
        mcuTimer()->tccrb.template add<Timer::tccrb_type::wgm2>();
        mcuInterrupts()->tifr.template reset<Timer::flags_type::ocfa | Timer::flags_type::ocfb>();
        mcuInterrupts()->timsk.template add<Timer::mask_type::ociea>();
    }
    
    inline static void init() {
        (Pins::low(), ...);
        (Pins::template dir<AVR::Output>(), ...);
        timerInit();
        constexpr auto medium = (parameter::ocMax + parameter::ocMin) / 2;
        std::iota(std::begin(ocrbValues), std::end(ocrbValues), medium, medium);
    }
    
    inline static void ppm(const std::percent& width, uint8_t channel) {
        assert(channel < numberOfChannels);
        uint16_t ocr = std::expand(width, parameter::ocMin, parameter::ocMax);
        uint16_t start = 0;
        for(uint8_t i = 0; i < channel; ++i) {
            start += ocrbValues[i];
        }
        uint16_t end = start + ocr;
        int16_t diff = ocr - ocrbValues[channel];
        {
            Scoped<DisbaleInterrupt<RestoreState>> di;
            ocrbValues[channel] = end;
            for(uint8_t i = channel + 1; i < numberOfChannels; ++i) {
                ocrbValues[i] += diff;
            }
        }
    }
    template<typename T, T Min, T Max>
    inline static uint16_t ppm(const uint_ranged<T, Min, Max>& v, uint8_t channel) {
        assert(channel < numberOfChannels);
        uint16_t ocr;
        if constexpr(std::is_same<ranged_type, uint_ranged<T, Min, Max>>::value) {
            ocr = v.toInt();
        }
        else {
            T v1 = v.toInt() - Min;
            constexpr uint64_t denom = Max - Min;
            constexpr uint64_t nom = ranged_type::Upper - ranged_type::Lower;
            ocr = Util::RationalDivider<uint16_t, nom, denom>::scale(v1) + ranged_type::Lower;
        }
        uint16_t start = 0;
        for(uint8_t i = 0; i < channel; ++i) {
            start += ocrbValues[i];
        }
        uint16_t end = start + ocr;
        int16_t diff = ocr - ocrbValues[channel];
        {
            Scoped<DisbaleInterrupt<RestoreState>> di;
            ocrbValues[channel] = end;
            for(uint8_t i = channel + 1; i < numberOfChannels; ++i) {
                ocrbValues[i] += diff;
            }
        }
        return ocr;
    }
    
    // todo: ermöglicht _vor_ ocrb-isr andere ISR (z.B.) uart abzuschalten
    struct OCAHandler : public IsrBaseHandler<typename AVR::ISR::Timer<Timer::number>::CompareA> {
        static void isr() {
            actual = 0;
            *mcuTimer()->ocrb = ocrbValues[0];
            mcuInterrupts()->timsk.template add<Timer::mask_type::ocieb>();
            first_pin::high();
        }
    };
    
    // todo: für PINs nur ocrb, und einschalten der abgeschalteten ISRs (s.a. ocra)
    struct OCBHandler : public IsrBaseHandler<typename AVR::ISR::Timer<Timer::number>::CompareB> {
        static void isr() {
            Meta::visitAt<pin_list>(actual, [](auto wrapper){
                decltype(wrapper)::type::off();
            });
            actual = (actual + 1) % numberOfChannels;
            if (actual != 0) {
                Meta::visitAt<pin_list>(actual, [](auto wrapper){
                    decltype(wrapper)::type::on();
                });
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



