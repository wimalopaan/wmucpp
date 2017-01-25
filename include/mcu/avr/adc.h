/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "hal/event.h"
#include "util/dassert.h"
#include "util/fixedpoint.h"

namespace AVR {

template<uint8_t N, typename MCU>
struct AdcParameter;

template<>
struct AdcParameter<0, ATMega1284P> {
    static constexpr EventType event = EventType::AdcConversion;
};

template<uint8_t N, typename MCU = DefaultMcuType>
class Adc final {
    static_assert(N < MCU::Adc::count, "wrong adc number"); 
    
public:
    static constexpr auto mcuAdc = getBaseAddr<typename MCU::Adc, N>;
    static constexpr uint8_t channelMask = 0x07;

    typedef uint16_t value_type;
    typedef FixedPoint<uint16_t, 8> voltage_type;
    
    static constexpr uint8_t bits = 10;
    static constexpr value_type value_mask = (1 << bits) - 1;

    static constexpr double VRef = 2.56;
    static constexpr double VBit = VRef / (1 << bits);
    
    typedef MCU mcu_type;
    static constexpr const uint8_t number = N;

    Adc() = delete;

    static void init() {
        mcuAdc()->admux = _BV(REFS1) | _BV(REFS0);
        mcuAdc()->adcsra = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
        mcuAdc()->adcsrb = 0;
    }
    
    static void startConversion() {
        assert(conversionReady());
        mcuAdc()->adcsra |= _BV(ADSC);
    }
    
    static bool conversionReady() {
        return !(mcuAdc()->adcsra & _BV(ADSC));
    }
    
    static uint16_t value() {
        return mcuAdc()->adc;
    }
    
    static FixedPoint<uint16_t, 8> toVoltage(uint16_t v) {
        return FixedPoint<uint16_t, 8>{v * VBit};
    }
    
    static void channel(uint8_t ch) {
        assert(ch < MCU::Adc::template Parameter<0>::numberOfChannels);
        mcuAdc()->admux = (mcuAdc()->admux & ~channelMask) | (ch & channelMask);
    }
};

}
