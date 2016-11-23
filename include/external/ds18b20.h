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

#include "std/array.h"
#include "std/algorithm.h"
#include "external/onewire.h"

// read-rom
// command (byte) read-ram (length)
// command (byte) write-ram (length)

template<typename OWDevice, const std::milliseconds& Period, typename CycleType = uint16_t, typename PeriodType = uint16_t>
class OWBitShifter final {
    enum class State : uint8_t {Undefined, Calibrate, Stop, ResetStart, ResetWait, QueryPresence, Write, BitHold, Read, BitSample};
    
    typedef CycleType cycle_type;
    typedef PeriodType period_type;
    typedef typename OWDevice::owmaster_type owmaster_type;
    
    static constexpr period_type startup_periods = 10;
    static constexpr period_type calibrate_periods = 20;
    
public:
    OWBitShifter() = delete;
    
    static void periodic() { // called every x-ms
        switch(mState) {
        case State::Undefined:
            ++mPeriod;
            if (mPeriod > startup_periods) {
                mCycle = 0;
                mCyclesPerPeriod = 0;
                mState = State::Calibrate;
            }
            break;
        case State::Calibrate:
            ++mPeriod;
            mCyclesPerPeriod = std::max(mCyclesPerPeriod, mCycle);
            mCycle = 0;
            if (mPeriod > calibrate_periods) {
                mCycleTime = std::duration_cast<std::microseconds>(Period) / mCyclesPerPeriod;
                mResetCycles = OneWire::Parameter<typename owmaster_type::mode_type>::reset / mCycleTime;           
                mPresenceCycles = OneWire::Parameter<typename owmaster_type::mode_type>::presenceAfterReset/ mCycleTime;           
                mState = State::Stop;
            }
            break;
        case State::Stop:
            break;
        default:
            break;
        }
    }
    static void freeRun() {
        switch(mState) {
        case State::Undefined:
            break;
        case State::Calibrate:
            ++mCycle;
            if (mCycle >= mBla) {
                mBla = mCycle;
            }
            break;
        case State::Stop:
            break;
        case State::ResetStart:
            mState = State::ResetWait;
            mCycle = 0;
            owmaster_type::Asynchronous::resetStart();
            break;
        case State::ResetWait:
            ++mCycle;
            if (mCycle >= mResetCycles) {
                owmaster_type::Asynchronous::resetStop();
                mState = State::QueryPresence;
                mCycle = 0;
            }
            break;
        case State::QueryPresence:
            ++mCycle;
            if (mCycle >= mPresenceCycles) {
                mPresence = owmaster_type::Asynchronous::isPresence();
                mCycle = 0;
                mState = State::Stop;
            }
            break;
        default:
            break;
        }
    }
    static void reset() {
        mState = State::ResetStart;
    }

//private:
    static volatile CycleType mBla;
    static bool mPresence;
    static CycleType mPresenceCycles;
    static CycleType mResetCycles;
    static std::microseconds mCycleTime;
    static CycleType mCyclesPerPeriod;
    static CycleType mCycle;
    static PeriodType mPeriod;
    static State mState;
};

template<typename OWDevice, const std::milliseconds& Period, typename CycleType, typename PeriodType>
volatile CycleType OWBitShifter<OWDevice, Period, CycleType, PeriodType>::mBla;
template<typename OWDevice, const std::milliseconds& Period, typename CycleType, typename PeriodType>
bool OWBitShifter<OWDevice, Period, CycleType, PeriodType>::mPresence = false;
template<typename OWDevice, const std::milliseconds& Period, typename CycleType, typename PeriodType>
CycleType OWBitShifter<OWDevice, Period, CycleType, PeriodType>::mPresenceCycles = 0;
template<typename OWDevice, const std::milliseconds& Period, typename CycleType, typename PeriodType>
CycleType OWBitShifter<OWDevice, Period, CycleType, PeriodType>::mResetCycles = 0;
template<typename OWDevice, const std::milliseconds& Period, typename CycleType, typename PeriodType>
std::microseconds OWBitShifter<OWDevice, Period, CycleType, PeriodType>::mCycleTime{0};
template<typename OWDevice, const std::milliseconds& Period, typename CycleType, typename PeriodType>
CycleType OWBitShifter<OWDevice, Period, CycleType, PeriodType>::mCyclesPerPeriod= 0;
template<typename OWDevice, const std::milliseconds& Period, typename CycleType, typename PeriodType>
CycleType OWBitShifter<OWDevice, Period, CycleType, PeriodType>::mCycle= 0;
template<typename OWDevice, const std::milliseconds& Period, typename CycleType, typename PeriodType>
PeriodType OWBitShifter<OWDevice, Period, CycleType, PeriodType>::mPeriod = 0;

template<typename OWDevice, const std::milliseconds& Period, typename CycleType, typename PeriodCounter>
typename OWBitShifter<OWDevice, Period, CycleType, PeriodCounter>::State 
OWBitShifter<OWDevice, Period, CycleType, PeriodCounter>::mState = OWBitShifter<OWDevice, Period, CycleType, PeriodCounter>::State::Undefined;


template<typename OneWireMaster, bool Single = true>
class DS18B20 final {
public:
    struct Command {
        static constexpr uint8_t readRom = 0x33;
        static constexpr uint8_t convert = 0x44;
        static constexpr uint8_t skipRom = 0xcc;
    };
    typedef OneWireMaster owmaster_type;
    
    static constexpr uint8_t romSize = 8;
    
    DS18B20() = delete;
    
    static void init() {
        OneWireMaster::Synchronous::reset();
    }
    
    static bool readRom(std::array<uint8_t, romSize>& rom) {
        if (!OneWireMaster::Synchronous::reset()) {
            return false;
        }
        OneWireMaster::Synchronous::write(Command::readRom);
        
        if constexpr(Single) {
            for(uint8_t i = 0; i < romSize; ++i) {
                rom[i] = OneWireMaster::Synchronous::read();
            }   
        }
        return true;
    }
private:
    static uint16_t stateCounter;
};
template<typename OneWireMaster, bool Single>
uint16_t DS18B20<OneWireMaster, Single>::stateCounter = 0;