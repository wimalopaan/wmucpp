/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "config.h"
#include "mcu/ports.h"
#include "mcu/avr/util.h"
#include "mcu/avr/delay.h"
#include "hal/event.h"
#include "container/fifo.h"
#include "util/disable.h"
#include "util/dassert.h"
#include "std/array.h"
#include "std/algorithm.h"
#include "container/pgmstring.h"

namespace OneWire {

struct Rom {
    template<typename Stream> friend Stream& operator<<(Stream& o, const Rom& rom);
    
    static constexpr uint8_t size = 8;
    uint8_t& operator[](uint8_t index) {
        return data[index];
    }
    const uint8_t& operator[](uint8_t index) const {
        return data[index];
    }
    explicit operator bool() const {
        return !std::all_of(std::begin(data), std::end(data), [](uint8_t v){return v == 0;}) && std::crc8(data);
    }
private:
    std::array<uint8_t, size> data;
};
typedef Rom ow_rom_t;

template<typename Stream>
Stream& operator<<(Stream& o, const Rom& rom) {
    o << "OneWireId["_pgm;
    for(const auto& v : rom.data) {
        o << v << ',';
    }
    o << static_cast<bool>(rom);
    o << ']';
    return o;
}

struct Normal {};
struct OverDrive{};

template<typename Mode>
struct Parameter;

template<>
struct Parameter<Normal> {
    static constexpr std::microseconds recovery = 20_us;
    static constexpr std::microseconds writeZeroTime = 60_us;
    static constexpr std::microseconds writeOneTime = 6_us;
    static constexpr std::microseconds readLow = 6_us;
    static constexpr std::microseconds sampleAfterPre = 9_us;
    static constexpr std::microseconds reset = 480_us;
    static constexpr std::microseconds presenceAfterReset= 70_us;
};

template<>
struct Parameter<OverDrive> {
    static constexpr std::centimicroseconds pre{10};
    static constexpr std::centimicroseconds zeroTotal{75};
    static constexpr std::centimicroseconds sampleAfterPre{10};
    static constexpr std::centimicroseconds reset{700};
    static constexpr std::centimicroseconds presenceAfterReset{85};
};

enum class Command {ReadRom = 0x33, SkipRom = 0xcc, MatchRom = 0x55, SearchRom = 0xf0, 
                    Convert = 0x44, ReadScratchpad = 0xbe, WriteScratchpad = 0x4e};


// fixme: asynchrone Schnittstelle wie bei I2C gestalten

template<typename OWMaster, const std::microseconds& delay, uint16_t BSize = 16>
class MasterAsync final {
    enum class State : uint8_t {Inactive, ResetWait, Sending, Reading};
    
    static_assert(delay >= OneWire::Parameter<OneWire::Normal>::reset, "delay too short");
    
public:
    typedef typename OWMaster::pin_type pin_type;
    typedef OneWire::Normal mode_type;
    static constexpr bool isAsync = true;
    
    MasterAsync() = delete;
    
    static void rateProcess() { // called every delay us
        static uint8_t data = 0;
        switch(mState) {
        case State::Inactive:
            if (!mSendQueue.empty()) {
                mState = State::Sending;
                mBitCount = 0;
                data = *mSendQueue.pop_front();
            }
            else if (mBytesToRead > 0) {
                mBitCount = 0;
                data = 0;
                mState = State::Reading;
            }
            break;
        case State::ResetWait:
        {
            Scoped<DisbaleInterrupt> di;
            Set<pin_type>::input();
            if constexpr(OWMaster::internal_pullup) {
                pin_type::pullup();
            }
            Util::delay(Parameter<mode_type>::presenceAfterReset);
            mDevicesPresent = !pin_type::read(); // actice low
        }
            mState = State::Inactive;
            break;
        case State::Sending:
            if (mBitCount++ < 8) {
                OWMaster::template writeBit<false>(data & 0x01);
                data >>= 1;
            }
            else {
                mState = State::Inactive;
            }
            break;
        case State::Reading:
            if (mBitCount++ < 8) {
                data >>= 1;
                if (OWMaster::template readBit<false>()) {
                    data |= 0x80;
                }
            }
            else {
                bool ok = mRecvQueue.push_back(data);
                assert(ok);
                --mBytesToRead;
                if (mBytesToRead == 0) {
                    EventManager::enqueue({EventType::OneWireRecvComplete, 0});
                    mState = State::Inactive;
                }
                else {
                    data = 0;
                    mBitCount = 0;
                }
            }
            break;
        default:
            break;
        }        
    } 
    static constexpr auto periodic = rateProcess;
    
    static bool reset() {
        pin_type::low();
        Set<pin_type>::output();
        mState = State::ResetWait;
        return true;
    }
    static void start() {
    }
    static void init() {
        OWMaster::init();
    }
    static bool put(uint8_t v) {
        return mSendQueue.push_back(v);
    }
    static std::optional<uint8_t> get() {
        return mRecvQueue.pop_front();
    }
    template<uint8_t N>
    static void startGet() {
        assert(mBytesToRead == 0);
        mBytesToRead = N;        
    }
    static bool devivesPresent() {
        return mDevicesPresent;
    }
private:
    static std::FiFo<uint8_t, BSize> mRecvQueue;
    static std::FiFo<uint8_t, BSize> mSendQueue;
    static uint8_t mBitCount;
    static State mState;
    static bool mDevicesPresent;
    static uint8_t mBytesToRead;
};
template<typename OWMaster, const std::microseconds& delay, uint16_t BSize>
std::FiFo<uint8_t, BSize> MasterAsync<OWMaster, delay, BSize>::mRecvQueue;
template<typename OWMaster, const std::microseconds& delay, uint16_t BSize>
std::FiFo<uint8_t, BSize> MasterAsync<OWMaster, delay, BSize>::mSendQueue;
template<typename OWMaster, const std::microseconds& delay, uint16_t BSize>
uint8_t MasterAsync<OWMaster, delay, BSize>::mBitCount = 0;
template<typename OWMaster, const std::microseconds& delay, uint16_t BSize>
typename MasterAsync<OWMaster, delay, BSize>::State MasterAsync<OWMaster, delay, BSize>::mState = MasterAsync<OWMaster, delay, BSize>::State::Inactive;
template<typename OWMaster, const std::microseconds& delay, uint16_t BSize>
bool MasterAsync<OWMaster, delay, BSize>::mDevicesPresent = false;
template<typename OWMaster, const std::microseconds& delay, uint16_t BSize>
uint8_t MasterAsync<OWMaster, delay, BSize>::mBytesToRead = 0;

template<typename Pin, typename Mode, bool InternalPullup = true>
class Master final {
public:
    Master() = delete;
    
    typedef Pin pin_type;
    typedef Mode mode_type;
    static constexpr bool internal_pullup = InternalPullup;
    static constexpr bool isAsync = false;
    
    static void init() {
        Set<Pin>::output();
        Pin::high();
    }

    template<bool UseRecovery = true>
    static void writeBit(bool bit) {
        if constexpr(UseRecovery) {
            Util::delay(Parameter<Mode>::recovery);
        }
        {
            Scoped<DisbaleInterrupt> di;
            Pin::low();
            Set<Pin>::output();
            Util::delay(Parameter<Mode>::writeOneTime); 
            if (bit) {
                Set<Pin>::input();
                Pin::pullup();      
            }
            Util::delay(Parameter<Mode>::writeZeroTime - Parameter<Mode>::writeOneTime);
        }
        Set<Pin>::input();
        Pin::pullup();      
    }    

    template<bool UseRecovery = true>
    static bool readBit() {
        if constexpr(UseRecovery) {
            Util::delay(Parameter<Mode>::recovery);
        }
        {
            Scoped<DisbaleInterrupt> di;
            Pin::low();
            Set<Pin>::output();
            Util::delay(Parameter<Mode>::readLow); 
            Set<Pin>::input();
            Pin::pullup();      
            Util::delay(Parameter<Mode>::sampleAfterPre);
            return Pin::read();
        }
    }
    static bool reset() {
        bool presence = false;
        Pin::low();
        Set<Pin>::output();
        Util::delay(Parameter<Mode>::reset);       
        {
            Scoped<DisbaleInterrupt> di;
            Set<Pin>::input();
            if constexpr(InternalPullup) {
                Pin::pullup();
            }
            Util::delay(Parameter<Mode>::presenceAfterReset);
            presence = !Pin::read(); // actice low
        }
        Util::delay(Parameter<Mode>::reset - Parameter<Mode>::presenceAfterReset);    
        return presence;
    }
    static uint8_t get() {
        uint8_t result = 0;
        for(uint8_t i = 0; i < 8; ++i) {
            result >>= 1;
            if (readBit()) {
                result |= 0x80;
            }
        }
        return result;
    }
    static void put(uint8_t byte) {
        for(uint8_t i = 0; i < 8; ++i) {
            writeBit(byte & 0x01);
            byte >>= 1;
        }        
    }
    static constexpr uint8_t LastDevice = 0x00;
    static constexpr uint8_t SearchFirst = 0xff;
    static constexpr uint8_t Error = 0xff;
    static constexpr uint8_t PresenceError = 0xfe;

    template<uint8_t MaxDevices>
    static uint8_t findDevices(std::array<ow_rom_t, MaxDevices>& devices) {
        auto diff = SearchFirst;
        for(uint8_t i = 0; i < devices.size; ++i) {
            diff = searchRom(diff, devices[i]);
            if ((diff == Error) || (diff == PresenceError)) {
                return i;
            }
            else if (diff == LastDevice) {
                return i + 1;
            }
        }
        return MaxDevices;
    }
    
    static uint8_t searchRom(uint8_t diff, ow_rom_t& rom) {
        uint8_t i = ow_rom_t::size * 8;
        uint8_t j = 8;
        uint8_t next_diff = LastDevice;
        bool  b = false;
        uint8_t* id = &rom[0];
        
        if(!reset()) {
            return PresenceError;
        }
        
        put(static_cast<uint8_t>(Command::SearchRom));
        
        do {
            j = 8;                          // 8 bits
            do {
                b = readBit();         // read bit
                if (readBit()) {      // read complement bit
                    if (b) {               // 0b11
                        return Error; // data error <--- early exit!
                    }
                }
                else {
                    if (!b) {              // 0b00 = 2 devices
                        if( (diff > i) || ((*id & 1) && diff != i) ) {
                            b = true;          // now 1
                            next_diff = i;  // next pass 0
                        }
                    }
                }
                writeBit(b);             // write bit
                *id >>= 1;
                if (b) {
                    *id |= 0x80;            // store bit
                }
                i--;
            } while(--j);
            id++;                           // next byte
        } while(i);
    
        return next_diff;                   // to continue search
    }
};

}