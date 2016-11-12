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

template<typename Buffer, typename Device, typename CounterType = uint8_t>
class ConstanteRateWriter {
public:
    static void rateProcess() {
    }
    static void reset() {
        counter = 0;
    }

private:
    static CounterType counter;
};
template<typename Buffer, typename Device, typename CounterType>
CounterType ConstanteRateWriter<Buffer, Device, CounterType>::counter = 0;


template<typename Timer, typename... Writers >
class ConstantRateAdapter {
public:
    static void periodic() {
        if (tickCounter > 0) {
            --tickCounter;
            (Writers::rateProcess(),...);
        }
    }
    static void start() {
        (Writers::reset(),...);
    }

    static void rateTick() {
        ++tickCounter;
    }
private:
    static uint8_t tickCounter;
};

template<typename Timer, typename... Writers>
uint8_t ConstantRateAdapter<Timer, Writers...>::tickCounter = 0;
