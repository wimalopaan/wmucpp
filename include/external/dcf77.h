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

#include <stdint.h>
#include <util/parity.h>

#include "std/time.h"
#include "mcu/ports.h"
#include "units/duration.h"
#include "units/physical.h"
#include "units/percent.h"
#include "hal/event.h"

using namespace std::literals::quantity;
template<typename Pin, const std::hertz& samplingFrequency, typename EventManager = void, bool Invert = false>
class DCF77 final  {
    static constexpr std::milliseconds zeroTime = 100_ms;
    static constexpr std::milliseconds oneTime = 200_ms;
    static constexpr std::milliseconds syncTime = 1000_ms;
    
    static constexpr auto numberOfSamplesForZero = zeroTime * samplingFrequency;
    static constexpr auto numberOfSamplesForOne  = oneTime * samplingFrequency;
    static constexpr auto numberOfSamplesForSync = syncTime * samplingFrequency;
    
    static constexpr auto threshZero = std::expand(80_ppc, decltype(numberOfSamplesForZero){0}, numberOfSamplesForZero);
    static constexpr auto threshOne  = std::expand(80_ppc, decltype(numberOfSamplesForOne){0}, numberOfSamplesForOne);
    static constexpr auto threshSync = numberOfSamplesForSync;
    static constexpr auto threshReset= threshOne;
    
    static constexpr uint8_t bitCounterMax = 60;
    static constexpr uint8_t NumberOfDCFBytes = 8;
    
    enum class State {Undefined, Low0, Low1, High, Sync, Error};
public:
    static void init() {
        Pin::template dir<AVR::Input>();
        Pin::pullup();
    }
    
    static void periodic() {
        static State state = State::Undefined;
        if constexpr(!Invert) {
            if (Pin::read()) {
                ++highCount;
            }
            else {
                ++lowCount;
            }
        }
        else {
            if (Pin::read()) {
                ++lowCount;
            }
            else {
                ++highCount;
            }
        }
        switch(state) {
        case State::High:
            if (lowCount >= threshZero) {
                state = State::Low0;
                highCount = 0;
            }
            if (highCount >= threshSync) {
                state = State::Sync;
                lowCount = 0;
                // Sync
                if (bitCounter >= 58) {
                    decodeDcfBits();
                    if constexpr(!std::is_same<EventManager, void>::value) {
                        EventManager::enqueue({EventType::DCFSync, 0});                
                    }
                }
                dcfBits.bits = 0;
                bitCounter = 0;
            }
            break;
        case State::Low0:
            if (lowCount >= threshOne) {
                state = State::Low1;
                highCount = 0;
            }
            if (highCount >= threshReset) {
                state = State::High;
                lowCount = 0;
                // Bit 0
                resetBit(bitCounter++);
            }
            break;
        case State::Low1:
            if (lowCount >= 2 * threshOne) {
                state = State::Error;
                lowCount = 0;
                highCount = 0;
            }
            if (highCount >= threshReset) {
                state= State::High;
                lowCount = 0;
                // Bit 1
                setBit(bitCounter++);
            }
            break;
        case State::Sync:
            if (lowCount >= threshZero) {
                state = State::Low0;
                highCount = 0;
            }
            if (highCount >= 2 * threshSync) {
                state = State::Error;
                lowCount = 0;
                highCount = 0;
            }
            break;
        case State::Error:
            if constexpr(!std::is_same<EventManager, void>::value) {
                EventManager::enqueue({EventType::DCFError, 0});                
            }
            lowCount = highCount = 0;
            state = State::Undefined;            

            [[fallthrough]];
        case State::Undefined:
            if (lowCount >= threshZero) {
                state = State::Low0;
                highCount = 0;
            }
            if (highCount >= threshReset) {
                state = State::High;
                lowCount = 0;
            }
            break;
        }

    }
    static const DateTime::TimeTm& dateTime() {
        return dcfTime;
    }

private:
    static bool checkDcfBits() {
        if (dcfBits.components.start != 0) {
            return false;
        }
        if (dcfBits.components.timeBegin != 1) {
            return false;
        }
        if (parity_even_bit((dcfBits.components.minute10 << 4) | dcfBits.components.minute1) != dcfBits.components.minuteP) {
            return false;
        }
        if (parity_even_bit((dcfBits.components.hour10 << 4) | dcfBits.components.hour1) != dcfBits.components.hourP) {
            return false;
        }
        if ((parity_even_bit((dcfBits.components.day10 << 4) | dcfBits.components.day1)
             ^ parity_even_bit((dcfBits.components.month10 << 4) | dcfBits.components.month1)
             ^ parity_even_bit((dcfBits.components.year10 << 4) | dcfBits.components.year1)
             ^ parity_even_bit(dcfBits.components.weekday)) != dcfBits.components.dateP) {
            return false;
        }
        return true;
    }
    
    static void decodeDcfBits() {
        if (checkDcfBits()) {
            dcfTime = DateTime::TimeTm(
                          DateTime::Day{(uint8_t)(dcfBits.components.day10 * 10 + dcfBits.components.day1)},
                          DateTime::Month{(uint8_t)(dcfBits.components.month10 * 10 + dcfBits.components.month1 - 1)},
                          DateTime::Year{(uint16_t)(dcfBits.components.year10 * 10 + dcfBits.components.year1 + 100)},
                          DateTime::Hour{(uint8_t)(dcfBits.components.hour10 * 10 + dcfBits.components.hour1)},
                          DateTime::Minute{(uint8_t)(dcfBits.components.minute10 * 10 + dcfBits.components.minute1)},
                          DateTime::Second{0},
                          dcfBits.components.dst    
                          ); 
        }   
        else {
            if constexpr(!std::is_same<EventManager, void>::value) {
                EventManager::enqueue({EventType::DCFParityError, 0});                
            }
        }
    }
    static DateTime::TimeTm dcfTime;
    static uint8_t highCount;
    static uint8_t lowCount;
    static uint8_t bitCounter;
    union DcfBits {
        uint64_t bits = 0;
        uint8_t  bytes[NumberOfDCFBytes];
        struct {
            uint64_t    start     : 1, 
                        weather   : 14, 
                        call      : 1,
                        dstSwitch : 1,
                        dst       : 1,
                        ndst      : 1,
                        addSec    : 1,
                        timeBegin : 1,
                        minute1   : 4,
                        minute10  : 3,
                        minuteP   : 1,
                        hour1     : 4,
                        hour10    : 2,
                        hourP     : 1,
                        day1      : 4,
                        day10     : 2,
                        weekday   : 3,
                        month1    : 4,
                        month10   : 1,
                        year1     : 4,
                        year10    : 4,
                        dateP     : 1,
                        unused    : 1;
        } __attribute__((packed)) components;
    };
    static DcfBits dcfBits;
    static inline void setBit(uint8_t number) {
        if (number < bitCounterMax) {
            dcfBits.bytes[number / 8] |= _BV(number % 8);
            if constexpr(!std::is_same<EventManager, void>::value) {
                EventManager::enqueue({EventType::DCFReceive1, number});                
            }
        }
    }
    static inline void resetBit(uint8_t number) {
        if (number < bitCounterMax) {
            dcfBits.bytes[number / 8] &= ~(_BV(number % 8));
            if constexpr(!std::is_same<EventManager, void>::value) {
                EventManager::enqueue({EventType::DCFReceive0, number});                
            }
        }
    }
};
template<typename Pin, const std::hertz& samplingFrequency, typename EM, bool Invert>
DateTime::TimeTm  DCF77<Pin, samplingFrequency, EM, Invert>::dcfTime;
template<typename Pin, const std::hertz& samplingFrequency, typename EM, bool Invert>
uint8_t DCF77<Pin, samplingFrequency, EM, Invert>::highCount = 0;
template<typename Pin, const std::hertz& samplingFrequency, typename EM, bool Invert>
uint8_t DCF77<Pin, samplingFrequency, EM, Invert>::lowCount = 0;
template<typename Pin, const std::hertz& samplingFrequency, typename EM, bool Invert>
uint8_t DCF77<Pin, samplingFrequency, EM, Invert>::bitCounter = 0;
template<typename Pin, const std::hertz& samplingFrequency, typename EM, bool Invert>
typename DCF77<Pin, samplingFrequency, EM, Invert>::DcfBits DCF77<Pin, samplingFrequency, EM, Invert>::dcfBits;