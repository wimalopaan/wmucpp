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
#include "compat/twi.h"
#include "mcu/avr8.h"
#include "mcu/avr/twiaddress.h"
#include "container/fifo.h"
#include "container/pgmstring.h"
#include "std/array.h"
#include "std/types.h"
#include "util/dassert.h"
#include "hal/event.h"

namespace TWI {

template<typename TWIMaster, uint8_t BSize = 16, bool UseSendEvent = false>
class MasterAsync final {
    enum class State : uint8_t {Inactive, StartWrite, StartWriteWait, WriteAddress, WriteAddressWait, Writing, 
                                WritingWait, Stop, StopWait, ReadAddressWait, Reading, ReadLast, ReadingWait, Error};
    static constexpr auto mcuTwi = TWIMaster::mcuTwi;
public:
    MasterAsync() = delete;
    static constexpr bool isAsync = true;
    typedef TWIMaster master_type;
    
    template<const std::hertz& fSCL>
    static void init() {
        TWIMaster::template init<fSCL>();    
    }

    static void rateProcess() {
        static State mState;
        static uint8_t lastAddress = 0;
        switch(mState) {
        case State::Inactive:
            if (!mSendQueue.empty()) {
                mState = State::StartWrite;                
            }
            break;
        case State::StartWrite:
            // send START condition
            mcuTwi()->twcr = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
            mState = State::StartWriteWait;                
            break;
        case State::StartWriteWait:
            if (TWIMaster::transmissionCompleted()) {
                // check value of TWI Status Register. Mask prescaler bits.
                uint8_t twst = (mcuTwi()->twsr & TW_STATUS_MASK) & 0xF8;
                if ( (twst != TW_START) && (twst != TW_REP_START)) {
                    mState = State::Error;
                    EventManager::enqueue({EventType::TWIError, 1});
                }
                else {
                    mState = State::WriteAddress;
                }
            }
            break;
        case State::WriteAddress:
            // send device address
            if (auto address = mSendQueue.pop_front()) {
                lastAddress = *address;
                mcuTwi()->twdr = *address;
                mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN);
                if (BusAddress<Write>::isWrite(*address)) {
                    mState = State::WriteAddressWait;
                }
                else {
                    mState = State::ReadAddressWait;
                }
            }
            else {
                mState = State::Error;
                EventManager::enqueue({EventType::TWIError, 2});
            }
            break;
        case State::ReadAddressWait:
            if (TWIMaster::transmissionCompleted()) {
                // check value of TWI Status Register. Mask prescaler bits.
                uint8_t twst = (mcuTwi()->twsr & TW_STATUS_MASK) & 0xF8;
                if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) {
                    mState = State::Error;
                    EventManager::enqueue({EventType::TWIError, 3});
                }
                else {
                    if (auto data = mSendQueue.pop_front()) {
                        mBytesToRead = *data;
                        if (mBytesToRead > 1) {
                            mState = State::Reading;
                        }
                        else {
                            mState = State::ReadLast;
                        }
                    }
                    else {
                        mState = State::Error;
                        EventManager::enqueue({EventType::TWIError, 4});
                    }
                }
            }
            break;
        case State::WriteAddressWait:
            if (TWIMaster::transmissionCompleted()) {
                // check value of TWI Status Register. Mask prescaler bits.
                uint8_t twst = (mcuTwi()->twsr & TW_STATUS_MASK) & 0xF8;
                if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) {
                    mState = State::Error;
                    EventManager::enqueue({EventType::TWIError, 5});
                }
                else {
                    mState = State::Writing;
                    if (auto data = mSendQueue.pop_front()) {
                        mBytesToWrite = *data;
                    }
                    else {
                        mState = State::Error;
                        EventManager::enqueue({EventType::TWIError, 6});
                    }
                }
            }
            break;
        case State::Writing:
            if (mBytesToWrite > 0) {
            // send data to the previously addressed device
                if (auto data = mSendQueue.pop_front()) {
                    mcuTwi()->twdr = *data;
                    mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN);
                    mState = State::WritingWait;
                }
            }
            else {
                mState = State::Stop;
                if constexpr(UseSendEvent) {
                    EventManager::enqueue({EventType::TWISendComplete, lastAddress >> 1});
                    lastAddress = 0;
                }
            }
            break;
        case State::WritingWait:
            if (TWIMaster::transmissionCompleted()) {
                // check value of TWI Status Register. Mask prescaler bits
                uint8_t twst = (mcuTwi()->twsr & TW_STATUS_MASK) & 0xF8;
                if (twst != TW_MT_DATA_ACK) {
                    mState = State::Error;
                    EventManager::enqueue({EventType::TWIError, 7});
                }
                else {
                    --mBytesToWrite;
                    mState = State::Writing;
                }
            }
            break;
        case State::Reading:
            mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
            mState = State::ReadingWait;
            break;
        case State::ReadLast:
            mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN) ;
            mState = State::ReadingWait;
            break;
        case State::ReadingWait:
            if(TWIMaster::transmissionCompleted()) {
                uint8_t v = mcuTwi()->twdr;
                --mBytesToRead;
                if (mRecvQueue.push_back(v)) {
                    if (mBytesToRead > 1) {
                        mState = State::Reading;
                    }
                    else if (mBytesToRead == 1){
                        mState = State::ReadLast;
                    }
                    else {
                        mState = State::Stop;
                        EventManager::enqueue({EventType::TWIRecvComplete, BusAddress<Write>::deviceAddressValue(lastAddress)});
                    }
                }   
                else {
                    mState = State::Error;
                    EventManager::enqueue({EventType::TWIError, 8});
                }
            } 
            break;
        case State::Stop:
            /* send stop condition */
             mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
            mState = State::StopWait;             
            break;
        case State::StopWait:
            // wait until stop condition is executed and bus released
            if (!(mcuTwi()->twcr & (1<<TWSTO))) {
                mState = State::Inactive;
            }
            break;
        case State::Error:
            mSendQueue.clear();
            mRecvQueue.clear();
            mState = State::Inactive;
            break;
        default:
            assert(false);
            break;
        }
    }
    static constexpr auto periodic = rateProcess;
   
    template<const Address& address>
    static bool startReadWithPointer(Range range) {
        bool ok = true;
        auto aw = BusAddress<Write>(address);
        ok &= mSendQueue.push_back(aw.value());
        ok &= mSendQueue.push_back(1);
        ok &= mSendQueue.push_back(range.pointer);
        auto ar = BusAddress<Read>(address);        
        ok &= mSendQueue.push_back(ar.value());
        ok &= mSendQueue.push_back(range.number);
        return ok;
    }
    template<const Address& address, uint8_t Pointer, uint8_t Number>
    static bool startReadWithPointer() {
        bool ok = true;
        auto aw = BusAddress<Write>(address);
        ok &= mSendQueue.push_back(aw.value());
        ok &= mSendQueue.push_back(1);
        ok &= mSendQueue.push_back(Pointer);
        auto ar = BusAddress<Read>(address);        
        ok &= mSendQueue.push_back(ar.value());
        ok &= mSendQueue.push_back(Number);
        return ok;
    }
    
    template<const Address& address, uint8_t L>
    static bool startWrite(const std::array<uint8_t, L>& data) {
        bool ok = true;
        auto a = BusAddress<Write>(address);
        ok &= mSendQueue.push_back(a.value());
        ok &= mSendQueue.push_back(data.size);
        for(uint8_t i = 0; i < data.size; ++i) {
            ok &= mSendQueue.push_back(data[i]);
        }
        return ok;
    }

    static std::optional<uint8_t> get() {
        return mRecvQueue.pop_front();
    }

private:
    static std::FiFo<uint8_t, BSize> mRecvQueue;
    static std::FiFo<uint8_t, BSize> mSendQueue;
    static uint_bounded<uint8_t> mBytesToRead;
    static uint_bounded<uint8_t> mBytesToWrite;
};
template<typename TWIMaster, uint8_t BSize, bool UseSendEvent>
uint_bounded<uint8_t> MasterAsync<TWIMaster, BSize, UseSendEvent>::mBytesToRead;
template<typename TWIMaster, uint8_t BSize, bool UseSendEvent>
uint_bounded<uint8_t> MasterAsync<TWIMaster, BSize, UseSendEvent>::mBytesToWrite;
template<typename TWIMaster, uint8_t BSize, bool UseSendEvent>
std::FiFo<uint8_t, BSize> MasterAsync<TWIMaster, BSize, UseSendEvent>::mRecvQueue;
template<typename TWIMaster, uint8_t BSize, bool UseSendEvent>
std::FiFo<uint8_t, BSize> MasterAsync<TWIMaster, BSize, UseSendEvent>::mSendQueue;

struct TwiSetupData {
    const uint8_t prescale = 0;
    const uint8_t twbr = 0;
};

template<typename TWI, uint8_t N>
constexpr TwiSetupData calculateNextPossibleBelow(const std::hertz& fTwi) {
    using pRow = typename TWI::template PrescalerRow<N>;
    for(const auto& p : pRow::values) {
        for(uint16_t twbr = 0; twbr <= 255; ++twbr) {
            uint32_t div = 16 + 2 * twbr * p;
            std::hertz f = Config::fMcu / div;
            if (f < fTwi) {
                return {p, (uint8_t)twbr}; 
            }
        }
    }
    return {};
}

template<uint8_t N, typename MCU = DefaultMcuType>
class Master final {
public:
    typedef MCU mcu_type;     
    typedef typename mcu_type::TWI mcu_twi_type;     
    static constexpr uint8_t number = N;
         
    Master() = delete;
    
    static constexpr bool isAsync = false;
    static constexpr const auto mcuTwi = AVR::getBaseAddr<typename MCU::TWI, N>;
    
    template<const std::hertz& fSCL>
    static void init() {
        constexpr auto tsd = calculateNextPossibleBelow<mcu_twi_type, N>(fSCL);
        mcuTwi()->twsr = MCU::TWI::template Prescaler<N, tsd.prescale>::value;                         
        mcuTwi()->twbr = tsd.twbr;  
    }

    static bool transmissionCompleted() {
        return mcuTwi()->twcr & (1<<TWINT);
    }
    
    static std::optional<Address> nextAddress(Address startAdress = TWI::minimumAddress) {
        for(auto address = startAdress; address <= TWI::maximumAddress; ++address) {
            if (start<Write>(address)) {
                return address;
            }
        }
        return {};
    }

    template<uint8_t MaxDevices>
    static uint8_t findDevices(std::array<Address, MaxDevices>& devices) {
        auto start = TWI::minimumAddress;
        for(uint8_t i = 0; i < devices.size; ++i) {
            if (auto next = nextAddress(start)) {
                devices[i] = *next;
                start = ++*next;
            }
            else {
                return i;
            }
        }
        return MaxDevices;
    }
    
    template<typename Mode>
    static bool start(Address deviceAddress) {
        return start(BusAddress<Mode>(deviceAddress));
    }
    
    template<typename Mode>
    static bool start(BusAddress<Mode> busAddress) {
        uint8_t twst = 0;
    
        // send START condition
        mcuTwi()->twcr = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
        // wait until transmission completed
        while(!transmissionCompleted());
    
        // check value of TWI Status Register. Mask prescaler bits.
        twst = (mcuTwi()->twsr & TW_STATUS_MASK) & 0xF8;
        if ( (twst != TW_START) && (twst != TW_REP_START)) {
            return false;
        }
    
        // send device address
        mcuTwi()->twdr = busAddress.value();
        mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN);
    
        // wail until transmission completed and ACK/NACK has been received
        while(!transmissionCompleted());
    
        // check value of TWI Status Register. Mask prescaler bits.
        twst = (mcuTwi()->twsr & TW_STATUS_MASK) & 0xF8;
        if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) {
            return false;
        }
        return true;
    }
    
    static void stop() {
        /* send stop condition */
         mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
         
         // wait until stop condition is executed and bus released
         while(mcuTwi()->twcr & (1<<TWSTO));
    }
    
    template<uint8_t L>
    static bool write(const std::array<uint8_t, L>& data, Address address) {
        bool ok = start<Write>(address);
        for(uint8_t i = 0; i < data.size; ++i) {
            ok &= write(data[i]);
        }
        stop();
        return ok;
    }
    
    template<const Address& address, uint8_t L>
    static bool write(const std::array<uint8_t, L>& data) {
        bool ok = start<Write>(address);
        for(uint8_t i = 0; i < data.size; ++i) {
            ok &= write(data[i]);
        }
        stop();
        return ok;
    }

    template<const Address& address, uint8_t Pointer, uint8_t L>
    static bool readWithPointer(std::array<uint8_t, L>& data) {
        bool ok = start<Write>(address);
        ok &= write<Pointer>();
        ok &= start<Read>(address);
        for(uint8_t i = 0; i < data.size - 1; ++i) {
            data[i] = read();
        }
        data[data.size - 1] = readBeforeStop();
        
        return ok;
    }

    template<const Address& address, uint8_t L>
    static bool readWithPointer(std::array<uint8_t, L>& data, uint8_t pointer) {
        bool ok = start<Write>(address);
        ok &= write(pointer);
        ok &= start<Read>(address);
        for(uint8_t i = 0; i < data.size - 1; ++i) {
            data[i] = read();
        }
        data[data.size - 1] = readBeforeStop();
        return ok;
    }
    
    static bool write(uint8_t data) {
        uint8_t   twst = 0;
        
        // send data to the previously addressed device
        mcuTwi()->twdr = data;
        mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN);
    
        // wait until transmission completed
        while(!transmissionCompleted());
    
        // check value of TWI Status Register. Mask prescaler bits
        twst = (mcuTwi()->twsr & TW_STATUS_MASK) & 0xF8;
        if( twst != TW_MT_DATA_ACK) return false;
        return true;       
    }
    template<uint8_t Data>
    static bool write() {
        uint8_t   twst = 0;
        
        // send data to the previously addressed device
        mcuTwi()->twdr = Data;
        mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN);
    
        // wait until transmission completed
        while(!transmissionCompleted());
    
        // check value of TWI Status Register. Mask prescaler bits
        twst = (mcuTwi()->twsr & TW_STATUS_MASK) & 0xF8;
        if( twst != TW_MT_DATA_ACK) return false;
        return true;       
    }
    
    static uint8_t read() {
        mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
        while(!transmissionCompleted());    
        return mcuTwi()->twdr;    
    }
    
    static uint8_t readBeforeStop() {
        mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN);
        while(!transmissionCompleted());
        return mcuTwi()->twdr;
    }
private:
};


}
