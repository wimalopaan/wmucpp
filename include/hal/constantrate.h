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
#include "util/bits.h"
#include "std/algorithm.h"
#include "units/physical.h"
#include "mcu/avr/isr.h"

template<typename Pin, uint8_t V>
class TestBitShifter {
public:
    static void rateProcess() {
        if (Util::isSet<Util::MSB>(value)) {
            Pin::high();
        }
        else {
            Pin::low();
        }
        value <<= 1;
    }
    static void start() {
        value = V;
    }
    static void reset() {
        value = V;
    }
    static void init() {
        Pin::template dir<AVR::Output>();
        Pin::low();
    }
private:
    static uint8_t value;
};
template<typename Pin, uint8_t V>
uint8_t TestBitShifter<Pin, V>::value = V;

template<typename Buffer, typename Device, typename CounterType = uint8_t>
class ConstanteRateWriter { 
public:
    static void rateProcess() {
        if (!mEnable) return;
        
        if (counter == 0) {
            Device::template rxEnable<false>();
        }
        if (counter < Buffer::size()) {
            if (auto data = Buffer::get(counter++)) {
                Device::put(*data);
            }
        }
        else {
            if (!Device::isEmpty()) {
                return;
            }
            else {
                Device::template rxEnable<true>();
            }
        }
    }
    static void enable(bool e) {
        mEnable = e;
    }

    static void start() {
        Buffer::reset();
        counter = 0;
    }
    static void init() {
        Buffer::init();
    }
private:
    static bool mEnable;
    static CounterType counter;
};
template<typename Buffer, typename Device, typename CounterType>
CounterType ConstanteRateWriter<Buffer, Device, CounterType>::counter = 0;
template<typename Buffer, typename Device, typename CounterType>
bool ConstanteRateWriter<Buffer, Device, CounterType>::mEnable = true;


template<typename Timer, typename... Writers >
class ConstantRateAdapter : public IsrBaseHandler<typename Timer::isr_type::CompareA> {
    template<typename... II> friend class IsrRegistrar;
public:
    static void periodic() {
        if (tickCounter > 0) {
            tickCounter = 0;
            (Writers::rateProcess(),...);
        }
    }
    static constexpr void init() {
        Timer::mcuTimer->tccrb |= _BV(WGM12);;
        Timer::mcuInterrupts->tifr  |= _BV(OCF1A) | _BV(OCF1B);
        Timer::mcuInterrupts->timsk |= _BV(OCIE0A);
        
        (Writers::init(),...);
    }
    static void start() {
        (Writers::start(),...);
    }
    static void rateTick() {
        ++tickCounter;
    }
    constexpr static auto isr = rateTick;
private:
    static uint8_t tickCounter;
};

template<typename Timer, typename... Writers>
uint8_t ConstantRateAdapter<Timer, Writers...>::tickCounter = 0;
