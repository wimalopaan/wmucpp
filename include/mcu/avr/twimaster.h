/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <cstdint>
#include <array>
#include "compat/twi.h"
#include "mcu/avr8.h"
#include "mcu/avr/twiaddress.h"
#include "mcu/avr/util.h"
#include "container/fifo.h"
#include "container/pgmstring.h"
#include "util/types.h"
#include "util/dassert.h"
#include "hal/event.h"

namespace TWI {
    template<bool UseSendEvent>
    class MasterAsyncBase {
    protected:
        inline static std::byte lastAddress{0};
    };
    template<>
    class MasterAsyncBase<false> {
    };
    
    template<typename TWIMaster, uint8_t BSize = 16, bool UseSendEvent = false, uint8_t RSize = BSize>
    class MasterAsync final : public MasterAsyncBase<UseSendEvent> {
        enum class State : uint8_t {Inactive, StartWrite, StartWriteWait, WriteAddress, WriteAddressWait, Writing, 
                                    WritingWait, Stop, StopWait, ReadAddressWait, Reading, ReadLast, ReadingWait, Error};
        static constexpr auto mcuTwi = TWIMaster::mcuTwi;
        using base = MasterAsyncBase<UseSendEvent>;
    public:
        MasterAsync() = delete;
        static constexpr bool isAsync = true;
        typedef TWIMaster master_type;
        static constexpr auto tw_status_mask = TWIMaster::tw_status_mask;
        typedef typename TWIMaster::tws tws;
        typedef typename TWIMaster::twc twc;
        
        template<const std::hertz& fSCL>
        inline static uint8_t init() {
            reset();
            return TWIMaster::template init<fSCL>();    
        }
        inline static void reset() {
            mSendQueue.clear();
            mState = State::Inactive;
        }        
        inline static bool active() {
            return mState != State::Inactive;
        }
        inline static bool transferComplete() {
            return (mBytesToRead == uint8_t{0}) && (mBytesToWrite == uint8_t{0});
        }
        inline static void rateProcess() {
            switch(mState) {
            case State::Inactive:
                if (!mSendQueue.empty()) {
                    mState = State::StartWrite;                
                }
                break;
            case State::StartWrite:
                // send START condition
                mcuTwi()->twcr.template set<twc::twint | twc::twsta | twc::twen>();
                //            mcuTwi()->twcr = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
                mState = State::StartWriteWait;                
                break;
            case State::StartWriteWait:
                if (TWIMaster::transmissionCompleted()) {
                    // check value of TWI Status Register. Mask prescaler bits.
                    auto twst = mcuTwi()->twsr.template get<tw_status_mask>();
                    //                uint8_t twst = (mcuTwi()->twsr & TW_STATUS_MASK) & 0xF8;
                    if ( (twst != tws::twStart) && (twst != tws::twRepStart)) {
                        //                if ( (twst != TW_START) && (twst != TW_REP_START)) {
                        mState = State::Error;
                        if constexpr(UseSendEvent) {
                            EventManager::enqueue({EventType::TWIError, std::byte{1}});
                        }
                    }
                    else {
                        mState = State::WriteAddress;
                    }
                }
                break;
            case State::WriteAddress:
                // send device address
                if (auto address = mSendQueue.pop_front()) {
                    if constexpr(UseSendEvent) {
                        base::lastAddress = *address;
                    }
                    *mcuTwi()->twdr = std::byte{*address};
                    mcuTwi()->twcr.template set<twc::twint | twc::twen>();
                    //                mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN);
                    if (BusAddress<Write>::isWrite(*address)) {
                        mState = State::WriteAddressWait;
                    }
                    else {
                        mState = State::ReadAddressWait;
                    }
                }
                else {
                    mState = State::Error;
                    if constexpr(UseSendEvent) {
                        EventManager::enqueue({EventType::TWIError, std::byte{2}});
                    }
                }
                break;
            case State::ReadAddressWait:
                if (TWIMaster::transmissionCompleted()) {
                    // check value of TWI Status Register. Mask prescaler bits.
                    //                uint8_t twst = (mcuTwi()->twsr & TW_STATUS_MASK) & 0xF8;
                    auto twst = mcuTwi()->twsr.template get<tw_status_mask>();
                    if ( (twst != tws::twMtSlaAck) && (twst != tws::twMrSlaAck) ) {
                        //                    if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) {
                        mState = State::Error;
                        if constexpr(UseSendEvent) {
                            EventManager::enqueue({EventType::TWIError, std::byte{3}});
                        }
                    }
                    else {
                        if (auto data = mSendQueue.pop_front()) {
                            mBytesToRead = std::to_integer<uint8_t>(*data);
                            if (mBytesToRead > uint8_t{1}) {
                                mState = State::Reading;
                            }
                            else {
                                mState = State::ReadLast;
                            }
                        }
                        else {
                            mState = State::Error;
                            if constexpr(UseSendEvent) {
                                EventManager::enqueue({EventType::TWIError, std::byte{4}});
                            }
                        }
                    }
                }
                break;
            case State::WriteAddressWait:
                if (TWIMaster::transmissionCompleted()) {
                    // check value of TWI Status Register. Mask prescaler bits.
                    auto twst = mcuTwi()->twsr.template get<tw_status_mask>();
                    //                uint8_t twst = (mcuTwi()->twsr & TW_STATUS_MASK) & 0xF8;
                    if ( (twst != tws::twMtSlaAck) && (twst != tws::twMrSlaAck) ) {
                        //                if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) {
                        mState = State::Error;
                        if constexpr(UseSendEvent) {
                            EventManager::enqueue({EventType::TWIError, std::byte{5}});
                        }
                    }
                    else {
                        mState = State::Writing;
                        if (auto data = mSendQueue.pop_front()) {
                            mBytesToWrite = std::to_integer<uint8_t>(*data);
                        }
                        else {
                            mState = State::Error;
                            if constexpr(UseSendEvent) {
                                EventManager::enqueue({EventType::TWIError, std::byte{6}});
                            }
                        }
                    }
                }
                break;
            case State::Writing:
                if (mBytesToWrite > uint8_t{0}) {
                    // send data to the previously addressed device
                    if (auto data = mSendQueue.pop_front()) {
                        *mcuTwi()->twdr = std::byte{*data};
                        mcuTwi()->twcr.template set<twc::twint | twc::twen>();
                        //                    mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN);
                        mState = State::WritingWait;
                    }
                }
                else {
                    mState = State::Stop;
                    if constexpr(UseSendEvent) {
                        EventManager::enqueue({EventType::TWISendComplete, std::byte{base::lastAddress >> 1}});
                        base::lastAddress = 0;
                    }
                }
                break;
            case State::WritingWait:
                if (TWIMaster::transmissionCompleted()) {
                    // check value of TWI Status Register. Mask prescaler bits
                    auto twst = mcuTwi()->twsr.template get<tw_status_mask>();
                    //                uint8_t twst = (mcuTwi()->twsr & TW_STATUS_MASK) & 0xF8;
                    if (twst != tws::twMtDataAck) {
                        //                if (twst != TW_MT_DATA_ACK) {
                        mState = State::Error;
                        if constexpr(UseSendEvent) {
                            EventManager::enqueue({EventType::TWIError, std::byte{7}});
                        }
                    }
                    else {
                        --mBytesToWrite;
                        mState = State::Writing;
                    }
                }
                break;
            case State::Reading:
                mcuTwi()->twcr.template set<twc::twint | twc::twen | twc::twea>();
                //            mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
                mState = State::ReadingWait;
                break;
            case State::ReadLast:
                mcuTwi()->twcr.template set<twc::twint | twc::twen>();
                //            mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN) ;
                mState = State::ReadingWait;
                break;
            case State::ReadingWait:
                if(TWIMaster::transmissionCompleted()) {
                    std::byte v = *mcuTwi()->twdr;
                    --mBytesToRead;
                    if (mRecvQueue.push_back(v)) {
                        if (mBytesToRead > uint8_t{1}) {
                            mState = State::Reading;
                        }
                        else if (mBytesToRead == uint8_t{1}){
                            mState = State::ReadLast;
                        }
                        else {
                            mState = State::Stop;
                            if constexpr(UseSendEvent) {
                                EventManager::enqueue({EventType::TWIRecvComplete, 
                                                   std::byte{BusAddress<Write>::deviceAddressValue(base::lastAddress)}});
                            }
                        }
                    }   
                    else {
                        mState = State::Error;
                        if constexpr(UseSendEvent) {
                            EventManager::enqueue({EventType::TWIError, std::byte{8}});
                        }
                    }
                } 
                break;
            case State::Stop:
                /* send stop condition */
                mcuTwi()->twcr.template set<twc::twint | twc::twen | twc::twsto>();
                //             mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
                mState = State::StopWait;             
                break;
            case State::StopWait:
                // wait until stop condition is executed and bus released
                if (!(mcuTwi()->twcr.template isSet<twc::twsto>())) {
                    //            if (!(mcuTwi()->twcr & (1<<TWSTO))) {
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
            ok &= mSendQueue.push_back(std::byte{1});
            ok &= mSendQueue.push_back(std::byte{range.pointer});
            auto ar = BusAddress<Read>(address);        
            ok &= mSendQueue.push_back(ar.value());
            ok &= mSendQueue.push_back(std::byte{range.number});
            if (!ok) {
                mSendQueue.clear();
            }
            return ok;
        }
        template<const Address& address, uint8_t Pointer, uint8_t Number>
        static bool startReadWithPointer() {
            bool ok = true;
            auto aw = BusAddress<Write>(address);
            ok &= mSendQueue.push_back(aw.value());
            ok &= mSendQueue.push_back(std::byte{1});
            ok &= mSendQueue.push_back(std::byte{Pointer});
            auto ar = BusAddress<Read>(address);        
            ok &= mSendQueue.push_back(ar.value());
            ok &= mSendQueue.push_back(std::byte{Number});
            if (!ok) {
                mSendQueue.clear();
            }
            return ok;
        }
        
        template<const Address& address, uint16_t L>
        static bool startWrite(const std::array<std::byte, L>& data) {
            bool ok = true;
            auto a = BusAddress<Write>(address);
            ok &= mSendQueue.push_back(a.value());
            ok &= mSendQueue.push_back(std::byte{data.size});
            for(uint8_t i = 0; i < data.size; ++i) {
                ok &= mSendQueue.push_back(data[i]);
            }
            if (!ok) {
                mSendQueue.clear();
            }
            return ok;
        }

        template<const Address& address, Util::Array C, typename... PType>
        static bool startWrite(const C& data, const PType&... prefixes) {
            static_assert((std::is_same<typename C::value_type, PType>::value && ... && true));
            bool ok = true;
            auto a = BusAddress<Write>(address);
            ok &= mSendQueue.push_back(a.value());
            ok &= mSendQueue.push_back(std::byte{data.size + sizeof...(prefixes)});
            if constexpr(sizeof...(prefixes) > 0) {
                (mSendQueue.push_back(prefixes), ...);
            }
            for(uint8_t i = 0; i < data.size; ++i) {
                ok &= mSendQueue.push_back(data[i]);
            }
            if (!ok) {
                mSendQueue.clear();
            }
            return ok;
        }
        
        static std::optional<std::byte> get() {
            return mRecvQueue.pop_front();
        }
    private:
        static inline State mState = State::Inactive;
        
        // todo: beide Queues zusammenlegen, weil entweder read oder write stattfindet.
        inline static std::FiFo<std::byte, BSize> mRecvQueue;
        inline static std::FiFo<std::byte, RSize> mSendQueue;
        inline static uint_bounded<uint8_t> mBytesToRead;
        inline static uint_bounded<uint8_t> mBytesToWrite;
    };
    
    struct TwiSetupData {
        const uint16_t prescale = 0;
        const uint8_t twbr = 0;
    };
    
    template<typename TWI, uint8_t N>
    constexpr TwiSetupData calculateNextPossibleBelow(const std::hertz& fTwi) {
        using pRow = typename TWI::template PrescalerRow<N>;
        auto pv = AVR::Util::prescalerValues(pRow::values);
        auto sortedp = ::Util::sort(pv);
        for(const auto& p : sortedp) {
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

    template<uint8_t N, typename MCU>
    struct TwiPorts;
    template<>
    struct TwiPorts<0, AVR::ATMega1284P> {
        using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
        typedef AVR::Pin<PortC, 0> scl;
        typedef AVR::Pin<PortC, 1> sda;
    };
    template<>
    struct TwiPorts<0, AVR::ATMega328PB> {
        using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
        typedef AVR::Pin<PortC, 5> scl;
        typedef AVR::Pin<PortC, 4> sda;
    };
    template<>
    struct TwiPorts<1, AVR::ATMega328PB> {
        using PortE = AVR::Port<DefaultMcuType::PortRegister, AVR::E>;
        typedef AVR::Pin<PortE, 1> scl;
        typedef AVR::Pin<PortE, 0> sda;
    };
    template<>
    struct TwiPorts<0, AVR::ATMega324PB> {
        using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
        typedef AVR::Pin<PortC, 0> scl;
        typedef AVR::Pin<PortC, 1> sda;
    };
    template<>
    struct TwiPorts<1, AVR::ATMega324PB> {
        using PortE = AVR::Port<DefaultMcuType::PortRegister, AVR::E>;
        typedef AVR::Pin<PortE, 6> scl;
        typedef AVR::Pin<PortE, 5> sda;
    };
    
    template<uint8_t N, typename MCU = DefaultMcuType>
    class Master final {
    public:
        typedef TwiPorts<N, MCU> ports;
        
        typedef MCU mcu_type;     
        typedef typename mcu_type::TWI      mcu_twi_type;     
        typedef typename mcu_type::TWI::TWS tws;     
        typedef typename mcu_type::TWI::TWC twc;     
        typedef typename MCU::TWI::template PrescalerRow<N> ps;
        static constexpr uint8_t number = N;
        
        Master() = delete;
        
        static constexpr bool isAsync = false;
        static constexpr const auto mcuTwi = AVR::getBaseAddr<typename MCU::TWI, N>;
        
        static constexpr auto tw_status_mask = tws::tws7 | tws::tws6 | tws::tws5 | tws::tws4 | tws::tws3;
        
        template<const std::hertz& fSCL>
        static uint8_t init() {
            if (uint8_t e = clear(); e != 0) {
                return e;
            }
            constexpr auto tsd = calculateNextPossibleBelow<mcu_twi_type, N>(fSCL);
            const auto bits = AVR::Util::bitsFrom<tsd.prescale>(ps::values);
            
            mcuTwi()->twsr.template set<bits>();          
            //        mcuTwi()->twsr = MCU::TWI::template Prescaler<N, tsd.prescale>::value;                         
            *mcuTwi()->twbr = tsd.twbr;  
            return 0;
        }
        
        static bool transmissionCompleted() {
            return mcuTwi()->twcr.template isSet<twc::twint>();
            //        return mcuTwi()->twcr & (1<<TWINT);
        }
        
        static std::optional<Address> nextAddress(Address startAdress = TWI::minimumAddress) {
            for(auto address = startAdress; address <= TWI::maximumAddress; ++address) {
                if (start<Write>(address)) {
                    return address;
                }
            }
            return {};
        }
        
        template<uint16_t MaxDevices>
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
            //        uint8_t twst = 0;
            
            // send START condition
            mcuTwi()->twcr.template set<twc::twint | twc::twsta | twc::twen>();
            //        mcuTwi()->twcr = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
            
            // wait until transmission completed
            while(!transmissionCompleted());
            
            // check value of TWI Status Register. Mask prescaler bits.
            auto twst = mcuTwi()->twsr.template get<tw_status_mask>();
            //        twst = (mcuTwi()->twsr & TW_STATUS_MASK) & 0xF8;
            if ( (twst != tws::twStart) && (twst != tws::twRepStart)) {
                //        if ( (twst != TW_START) && (twst != TW_REP_START)) {
                return false;
            }
            
            // send device address
            *mcuTwi()->twdr = busAddress.value();
            mcuTwi()->twcr.template set<twc::twint | twc::twen>();
            //        mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN);
            
            // wail until transmission completed and ACK/NACK has been received
            while(!transmissionCompleted());
            
            // check value of TWI Status Register. Mask prescaler bits.
            twst = mcuTwi()->twsr.template get<tw_status_mask>();
            //        twst = (mcuTwi()->twsr & TW_STATUS_MASK) & 0xF8;
            if ( (twst != tws::twMtSlaAck) && (twst != tws::twMrSlaAck) ) {
                //        if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) {
                return false;
            }
            return true;
        }

        static uint8_t clear() {
            mcuTwi()->twcr.template clear<twc::twen>();
            ports::scl::template dir<AVR::Input>();
            ports::sda::template dir<AVR::Input>();
            ports::scl::off();
            ports::sda::off();
            
            Util::delay(100_ms);
            
            if (!ports::scl::isHigh()) {
                return 1;
            }
            
            bool sda_low = !ports::sda::isHigh();
            uint8_t clockCount = 20;
            while(sda_low && (clockCount > 0)) {
                --clockCount;
                ports::scl::template dir<AVR::Output>();
                Util::delay(10_us);
                ports::scl::template dir<AVR::Input>();
                Util::delay(10_us);
                bool scl_low = !ports::scl::isHigh();
                uint8_t count = 20;
                while(scl_low && (count > 0)) {
                    --count;
                    Util::delay(10_ms);
                    scl_low = !ports::scl::isHigh();
                }
                if (scl_low) {
                    return 2;
                } 
                sda_low = !ports::sda::isHigh();
            }
            if (sda_low) {
                return 3;
            }
            ports::sda::template dir<AVR::Output>();
            Util::delay(10_us);
            ports::sda::template dir<AVR::Input>();
            
            return 0;
        }        
        static void stop() {
            /* send stop condition */
            mcuTwi()->twcr.template set<twc::twint | twc::twen | twc::twsto>();
            //         mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
            
            // wait until stop condition is executed and bus released
            while(mcuTwi()->twcr.template isSet<twc::twsto>());
            //         while(mcuTwi()->twcr & (1<<TWSTO));
        }
        
        template<uint16_t L>
        static bool write(const std::array<std::byte, L>& data, Address address) {
            bool ok = start<Write>(address);
            for(uint8_t i = 0; i < data.size; ++i) {
                ok &= write(data[i]);
            }
            stop();
            return ok;
        }
        
        template<const Address& address, uint16_t L>
        static bool write(const std::array<std::byte, L>& data) {
            bool ok = start<Write>(address);
            for(uint8_t i = 0; i < data.size; ++i) {
                ok &= write(data[i]);
            }
            stop();
            return ok;
        }
        
//        template<const Address& address, auto L, typename... PType>
//        static bool write(const char (&data)[L], const PType&... prefixes) {
//            bool ok = start<Write>(address);
//            for(uint8_t i = 0; i < L; ++i) {
//                ok &= write(std::byte{data[i]});
//            }
//            stop();
//            return ok;
//        }

        template<const Address& address, Util::Array C, typename... PType>
        static bool write(const C& data, const PType&... prefixes) {
            bool ok = start<Write>(address);
            if constexpr(sizeof...(prefixes) > 0) {
                ((ok &= write(prefixes)), ...);
            }
            for(typename C::size_type i = 0; i < data.size; ++i) {
                ok &= write(std::byte{data[i]});
            }
            stop();
            return ok;
        }
        
        template<const Address& address, uint8_t Pointer, uint16_t L>
        static bool readWithPointer(std::array<std::byte, L>& data) {
            bool ok = start<Write>(address);
            ok &= write<Pointer>();
            ok &= start<Read>(address);
            for(uint8_t i = 0; i < data.size - 1; ++i) {
                data[i] = read();
            }
            data[data.size - 1] = readBeforeStop();
            
            return ok;
        }
        
        template<const Address& address, uint16_t L>
        static bool readWithPointer(std::array<std::byte, L>& data, uint8_t pointer) {
            bool ok = start<Write>(address);
            ok &= write(std::byte{pointer});
            ok &= start<Read>(address);
            for(uint8_t i = 0; i < data.size - 1; ++i) {
                data[i] = read();
            }
            data[data.size - 1] = readBeforeStop();
            return ok;
        }
        
        static bool write(std::byte  data) {
            //        uint8_t   twst = 0;
            
            // send data to the previously addressed device
            *mcuTwi()->twdr = data;
            mcuTwi()->twcr.template set<twc::twint | twc::twen>();
            //        mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN);
            
            // wait until transmission completed
            while(!transmissionCompleted());
            
            // check value of TWI Status Register. Mask prescaler bits
            //        twst = (mcuTwi()->twsr & TW_STATUS_MASK) & 0xF8;
            auto twst = mcuTwi()->twsr.template get<tw_status_mask>();
            if( twst != tws::twMtDataAck) return false;
            //        if( twst != TW_MT_DATA_ACK) return false;
            return true;       
        }
        template<uint8_t Data>
        static bool write() {
            //        uint8_t   twst = 0;
            
            // send data to the previously addressed device
            *mcuTwi()->twdr = std::byte{Data};
            mcuTwi()->twcr.template set<twc::twint | twc::twen>();
            //        mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN);
            
            // wait until transmission completed
            while(!transmissionCompleted());
            
            // check value of TWI Status Register. Mask prescaler bits
            auto twst = mcuTwi()->twsr.template get<tw_status_mask>();
            //        twst = (mcuTwi()->twsr & TW_STATUS_MASK) & 0xF8;
            if( twst != tws::twMtDataAck) return false;
            //        if( twst != TW_MT_DATA_ACK) return false;
            return true;       
        }
        
        static std::byte read() {
            mcuTwi()->twcr.template set<twc::twint | twc::twen | twc::twea>();
            //        mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
            while(!transmissionCompleted());    
            return *mcuTwi()->twdr;    
        }
        
        static std::byte readBeforeStop() {
            mcuTwi()->twcr.template set<twc::twint | twc::twen>();
            //        mcuTwi()->twcr = (1<<TWINT) | (1<<TWEN);
            while(!transmissionCompleted());
            return *mcuTwi()->twdr;
        }
    private:
    };
    
    
}
