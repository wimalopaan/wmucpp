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
#include "mcu/avr/isr.h"
#include "units/physical.h"

namespace AVR {

enum class TimerMode : uint8_t {Normal = 0, CTC, NumberOfModes};

template<typename Timer, uint8_t N, TimerMode Mode>
struct TimerFlags {
    // todo: umstellen   
};

template<typename MCU, uint8_t N>
struct TimerBase
{
    TimerBase() = delete;
    static constexpr auto mcuInterrupts = getBaseAddr<typename MCU::TimerInterrupts, N>;
    typedef MCU mcu_type;
};

template<uint8_t N, typename MCU = DefaultMcuType>
class Timer8Bit : public TimerBase<MCU, N> {
public:
    // todo: umstellen auf anderen zähler
//    static_assert(N < MCU::Timer8Bit::count, "wrong number");
    static constexpr uint8_t number = N;
    static constexpr auto mcuTimer = getBaseAddr<typename MCU::Timer8Bit, N>;
    typedef typename MCU::Timer8Bit mcu_timer_type;
    typedef uint8_t value_type;

    static constexpr const bool hasOcrA = true;
    static constexpr const bool hasOcrB = true;
    static constexpr const bool hasOverflow = true;

    Timer8Bit() = delete;

    template<int PreScale>
    static void prescale() {
        mcuTimer()->tccrb |= MCU::Timer8Bit::template Prescaler<N, PreScale>::value;
    }

    static void start(){
    }

    static void ocra(uint8_t v) {
        mcuTimer()->ocra = v;
    }
    template<uint8_t V>
    static void ocra() {
        mcuTimer()->ocra = V;
    }

    // todo: template
    static void mode(const TimerMode& mode) {
        if (mode == TimerMode::CTC) {
            TimerBase<MCU, N>::mcuInterrupts()->timsk |= MCU::TimerInterrupts::template Flags<N>::ociea;
            mcuTimer()->tccra = MCU::Timer8Bit::template Flags<N>::wgm1;
        }
        else if (mode == TimerMode::Normal) {
        }
    }
};

template<uint8_t N>
struct Timer8Bit<N, ATMega8> : public TimerBase<ATMega8, N>
{
    static constexpr uint8_t number = N;
    static constexpr auto mcuTimer = getBaseAddr<typename ATMega8::Timer8Bit, N>;
    typedef typename ATMega8::Timer8Bit mcu_timer_type;
    typedef uint8_t value_type;

    static constexpr const bool hasOcrA = false;
    static constexpr const bool hasOcrB = false;
    static constexpr const bool hasOverflow = true;
    
    Timer8Bit() = delete;

    template<int PreScale>
    static constexpr void prescale() {
        mcuTimer()->tccr |= ATMega8::Timer8Bit::template Prescaler<PreScale>::value;
    }
    static void start(){
    }
    
    static void mode(const TimerMode& mode) {
        if (mode == TimerMode::CTC) {
//            TimerBase<ATMega8, N>::mcuInterrupts->timsk |= ATMega8::TimerInterrupts::template Flags<N>::ociea;
            mcuTimer()->tccr = ATMega8::Timer8Bit::template Flags<N>::wgm1;
        }
        else if (mode == TimerMode::Normal) {
        }
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
    typedef AVR::ISR::Timer<N> isr_type;
    static constexpr uint8_t number = N;
    static constexpr auto mcuTimer = getBaseAddr<typename MCU::Timer16Bit, N>;
    typedef typename MCU::Timer16Bit mcu_timer_type;
    typedef uint16_t value_type;

    static constexpr const bool hasOcrA = true;
    static constexpr const bool hasOcrB = true;
    static constexpr const bool hasOverflow = true;

    Timer16Bit() = delete;

    template<int PreScale>
    static void prescale() {
        mcuTimer()->tccrb |= MCU::Timer16Bit::template Prescaler<PreScale>::value;
    }
    
    static void ocra(uint16_t v) {
        mcuTimer()->ocra = v;
    }
    template<uint16_t V>
    static void ocra() {
        mcuTimer()->ocra = V;
    }
    
    static void start(){
    }

    // todo: template  
    // modi: CTC, OCIEA, OCIEB, ...
    
    template<TimerMode Mode>
    static void mode() {
        mcuTimer()->tccrb = TimerFlags<Timer16Bit, N, Mode>::value;
    }

    static void mode(const TimerMode& mode) {
        if (mode == TimerMode::CTC) {
//            TimerBase<MCU, N>::mcuInterrupts->timsk |= _BV(OCIE0A);
            TimerBase<MCU, N>::mcuInterrupts()->timsk |= MCU::TimerInterrupts::template Flags<N>::ociea;
            mcuTimer()->tccrb |= _BV(WGM12);
        }
    }
};

}
