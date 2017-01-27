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

#include "mcu/avr/adc.h"
#include "util/dassert.h"

template<typename MCUAdc, uint8_t... Channels>
class AdcController final {
    enum class State {Start, Converting, ConversionComplete};
    
public:
    typedef MCUAdc mcu_adc_type;
    
    static constexpr uint8_t channels[] = {Channels...};
    static constexpr uint8_t NumberOfChannels = sizeof... (Channels);    
    static void init() {
        MCUAdc::init();
        MCUAdc::channel(channels[0]);
    }

    static void periodic() {
        static State state = State::Start;
        switch(state) {
        case State::Start:
            MCUAdc::startConversion();
            state = State::Converting;
            break;
        case State::Converting:
            if (MCUAdc::conversionReady()) {
                state = State::ConversionComplete;
            }
            break;
        case State::ConversionComplete:
            values[mActualChannel] = MCUAdc::value();
            mActualChannel = (mActualChannel + 1) % NumberOfChannels;
            MCUAdc::channel(channels[mActualChannel]);
            state = State::Start;
            break;
        default:
            assert(false);
            break;
        }
    }
    
    static typename MCUAdc::value_type value(uint8_t index) {
        assert(index < NumberOfChannels);
        return values[index];
    }

    static typename MCUAdc::voltage_type voltage(uint8_t index) {
        return MCUAdc::toVoltage(value(index));
    }

private:
    static typename MCUAdc::value_type values[NumberOfChannels];
    static uint8_t mActualChannel;
};

template<typename MCUAdc, uint8_t... Channels>
typename MCUAdc::value_type AdcController<MCUAdc, Channels...>::values[NumberOfChannels] = {};
template<typename MCUAdc, uint8_t... Channels>
uint8_t AdcController<MCUAdc, Channels...>::mActualChannel = 0;