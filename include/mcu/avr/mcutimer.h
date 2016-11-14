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

#include <stdint.h>

#include "mcu/avr8.h"
#include "units/physical.h"

namespace AVR {

enum class TimerMode : uint8_t {Normal = 0,
                                CTC
                               };

template<typename MCU, uint8_t N>
struct TimerBase
{
    TimerBase() = delete;
    static constexpr auto mcuInterrupts = getBaseAddr<typename MCU::TimerInterrupts, N>();
    typedef MCU mcu_type;
};

template<uint8_t N, typename MCU = DefaultMcuType>
class Timer8Bit : public TimerBase<MCU, N> {
public:
    static constexpr uint8_t number = N;
    static constexpr auto mcuTimer = getBaseAddr<typename MCU::Timer8Bit, N>();
    typedef typename MCU::Timer8Bit mcu_timer_type;
    typedef uint8_t value_type;

    Timer8Bit() = delete;

    template<int PreScale>
    static void prescale() {
        mcuTimer->tccrb |= MCU::Timer8Bit::template Prescaler<N, PreScale>::value;
    }

    static void start(){
    }

    static void ocra(uint8_t v) {
        mcuTimer->ocra = v;
    }
    static void mode(const TimerMode& mode) {
        if (mode == TimerMode::CTC) {
            TimerBase<MCU, N>::mcuInterrupts->timsk |= _BV(OCIE0A);
            mcuTimer->tccra = _BV(WGM01);
        }
        else if (mode == TimerMode::Normal) {
        }
    }
private:
};

template<uint8_t N>
struct Timer8Bit<N, ATMega8> : public TimerBase<ATMega8, N>
{
    static constexpr auto mcuTimer = getBaseAddr<typename ATMega8::Timer8Bit, N>();
    typedef uint8_t value_type;

    Timer8Bit() = delete;

    template<int PreScale>
    static void prescale() {
        mcuTimer->tccr |= ATMega8::Timer8Bit::template Prescaler<PreScale>::value;
    }
    static void start(){
    }
};

template<uint8_t N>
struct Timer8Bit<N, ATMegaNone> : public TimerBase<ATMegaNone, N>
{
    template<int PreScale>
    static void prescale() {
    }
    static void start(){
    }
};

template<uint8_t N, typename MCU = DefaultMcuType>
struct Timer16Bit: public TimerBase<MCU, N>
{
    static constexpr uint8_t number = N;
    static constexpr auto mcuTimer = getBaseAddr<typename MCU::Timer16Bit, N>();
    typedef typename MCU::Timer16Bit mcu_timer_type;
    typedef uint16_t value_type;

    Timer16Bit() = delete;

    template<int PreScale>
    static constexpr void prescale() {
        mcuTimer->tccrb |= MCU::Timer16Bit::template Prescaler<PreScale>::value;
    }
    
    static void start(){
    }
    // todo: template  
    // modi: CTC, OCIEA, OCIEB, ...
    static void mode(const TimerMode& mode) {
        if (mode == TimerMode::CTC) {
            TimerBase<MCU, N>::mcuInterrupts->timsk |= _BV(OCIE0A);
        }
    }
};

}
