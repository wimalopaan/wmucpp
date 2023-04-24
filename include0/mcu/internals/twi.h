#pragma once

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
#include <type_traits>

#include <etl/fifo.h>
#include <etl/fixedvector.h>
#include <etl/concepts.h>

#include "portmux.h"

namespace AVR {
    
    namespace Twi {
        struct Write{};
        struct Read{};
        
        struct Address {
            static constexpr uint8_t lowest = 0x08;
            static constexpr uint8_t highest = 0x77;
            
            explicit constexpr Address(uint8_t a) : value{a}{}
            
            using value_type = etl::uint_ranged<uint8_t, lowest, highest>; 
            const value_type value{};  
        };
        
        struct BusAddress {
            constexpr BusAddress() = default;
            explicit constexpr BusAddress(const Address a, const Write) : 
                mValue(std::byte(a.value << 1)){}
            explicit constexpr BusAddress(const Address a, const Read) : 
                mValue(std::byte(a.value << 1) | 0x01_B){}
            std::byte mValue{0};    
        };
        
        template<typename CN, typename Size = etl::NamedConstant<16>, typename MCU = DefaultMcuType>
        struct Master;
        
        template<AVR::Concepts::ComponentPosition CP, etl::Concepts::Container C, typename MCU>
        struct Master<CP, C, MCU> final {
            enum class State : uint8_t {Undefined, Idle, 
                                        TransferReadAdr, TransferReadWait, TransferReadData, TransferReadStop, TransferReadComplete,
                                        TransferWriteAdr, TransferWriteWait, TransferWriteData, 
                                        Stop,
                                        WaitToBeRestarted,
                                        Error
                                       };
            
            static inline constexpr auto N = CP::component_type::value;
            
            static constexpr auto mcu_twi = getBaseAddr<typename MCU::Twi, N>;
            
            using mctrla_t = MCU::Twi::MCtrlA_t;
            using mctrlb_t = MCU::Twi::MCtrlB_t;
            using mstatus_t = MCU::Twi::MStatus_t;
            
            inline static constexpr size_t size() {
                return C::size();
            }
            static inline void address(const Address adr) {
                mBA = BusAddress(adr, Write{});
            }            
            template<typename T, T... Ts>
            inline static bool write(const AVR::Pgm::Array<T, Ts...>& data) {
                static_assert(sizeof...(Ts) <= size());
                if (mState != State::Idle) {
                    return false;
                }
                for(uint8_t i{}; i < data.size(); ++i) {
                    mData[i] = data[i];
                }
                mState = State::TransferWriteAdr;
                mMaxBytes = data.size();
                return true;
                
            }
            static inline void init() {
                *mcu_twi()->masterBaudrate = baud;
                mcu_twi()->mctrla.template set<mctrla_t::enable>();
                mcu_twi()->mstatus.template reset<mstatus_t::busStateIdle>();
            }
            static inline bool isIdle() {
                return mState == State::Idle;    
            }
            static inline bool isWaiting() {
                return mState == State::WaitToBeRestarted;    
            }
            static inline bool isReady() {
                return isIdle() || isWaiting();    
            }
            static inline void restart() {
                if (mState == State::WaitToBeRestarted) {
                    mState = State::Idle;
                }
            }
            
            static inline void periodic() {
                const auto oldState = mState;
                switch(mState) {
                case State::Undefined:
                    mState = State::Idle;
                break;
                case State::Idle:
                break;
                case State::TransferReadAdr:
                    *mcu_twi()->masterAddress = mBA.mValue;
                    mState = State::TransferReadWait;
                break;
                case State::TransferReadWait:
                    if (mcu_twi()->mstatus.template isSet<mstatus_t::readIF | mstatus_t::writeIF>()) {
                        if (mcu_twi()->mstatus.template isSet<mstatus_t::rxAck>()) {
                            mState = State::Error;
                        }
                        else {
                            mState = State::TransferReadData;
                        }
                    };
                    mcu_twi()->mstatus.template testAndReset<mstatus_t::busErr | mstatus_t::arbLost>([]{
                        mState = State::Error;
                    });
                break;
                case State::TransferReadData:
                    lastReadData = *mcu_twi()->masterData;
                    mState = State::TransferReadStop;
                    mcu_twi()->mstatus.template testAndReset<mstatus_t::busErr | mstatus_t::arbLost>([]{
                        mState = State::Error;
                    });
                break;
                case State::TransferReadStop:
                    mcu_twi()->mctrlb.template set<mctrlb_t::MStop | mctrlb_t::ackact>();
                    mState = State::TransferReadComplete;
                break;
                case State::TransferReadComplete:
                break;
                case State::TransferWriteAdr:
                    *mcu_twi()->masterAddress = mBA.mValue;
                    mState = State::TransferWriteWait;
                    mBytesTransferred = 0;
                break;
                case State::TransferWriteWait:
                    mcu_twi()->mstatus.template testAndReset<mstatus_t::readIF | mstatus_t::writeIF>([]{
                        if (mcu_twi()->mstatus.template isSet<mstatus_t::rxAck>()) {
                            mState = State::Error;
                        }
                        else {
                            mState = State::TransferWriteData;
                        }
                    });
                    mcu_twi()->mstatus.template testAndReset<mstatus_t::busErr | mstatus_t::arbLost>([]{
                        mState = State::Error;
                    });
                break;
                case State::TransferWriteData:
                    if (mBytesTransferred < mMaxBytes) {
                        *mcu_twi()->masterData = mData[mBytesTransferred++];
                        mState = State::TransferWriteWait;
                    }
                    else {
                        mState = State::Stop;
                    }
                break;
                case State::Stop:
                    mcu_twi()->mctrlb.template set<mctrlb_t::MStop>();
                    if (mMaxBytes == mData.size()) {
                        mState = State::Idle;
                    }
                    else {
                        mState = State::WaitToBeRestarted;
                    }
                break;
                case State::Error:
                    mcu_twi()->mctrlb.template set<mctrlb_t::MStop>();
                    mcu_twi()->mstatus.template reset<mstatus_t::busStateIdle>();
                    mState = State::Idle;
                break;
                case State::WaitToBeRestarted:
                break;
                }
                if (oldState != mState) {
                    switch(mState) {
                    case State::Undefined:
                    break;
                    case State::Idle:
                    break;
                    case State::TransferReadAdr:
                    break;
                    case State::TransferReadWait:
                    break;
                    case State::TransferReadData:
                    break;
                    case State::TransferReadStop:
                    break;
                    case State::TransferReadComplete:
                    break;
                    case State::TransferWriteAdr:
                    break;
                    case State::TransferWriteWait:
                    break;
                    case State::TransferWriteData:
                    break;
                    case State::WaitToBeRestarted:
                    break;
                    case State::Stop:
                    break;
                    case State::Error:
                    break;
                    }
                }
            }
            static inline State state() {
                return mState;                    
            }
            static inline void ratePeriodic() {
                if (mState == State::Idle) {
                    mState = State::TransferWriteAdr;
                    mMaxBytes = mData.size();
                }                
            }
            static inline C& data() {
                return mData;
            }
        private:
//            C::size_type::_;
            inline static BusAddress mBA;
            inline static C mData;
//            inline static decltype(std::begin(mData)) mWriteIterator;
            inline static C::size_type mMaxBytes{};
            inline static C::size_type mBytesTransferred{};
            inline static std::byte lastReadData{};
            inline static State mState{State::Undefined};
            
            inline static constexpr uint8_t baud = []{
                constexpr External::Units::hertz fTwi{400'000};
                constexpr uint16_t b = Project::Config::fMcu / (2 * fTwi) - 5;
                static_assert(b <= 255);
                return b;
            }();
        };
        
        
        template<AVR::Concepts::ComponentPosition CP, auto Size, typename MCU>
        struct Master<CP, etl::NamedConstant<Size>, MCU> final {
            enum class State : uint8_t {Undefined, Idle, 
                                        TransferReadAdr, TransferReadWait, TransferReadData, TransferReadStop, TransferReadComplete,
                                        TransferWriteAdr, TransferWriteWait, TransferWriteData, 
                                        Stop,
                                        Error
                                       };
            
            
            static inline constexpr uint8_t size() {
                return Size;
            }
            
            static inline constexpr auto N = CP::component_type::value;
            
            static constexpr auto mcu_twi = getBaseAddr<typename MCU::Twi, N>;
            
            using mctrla_t = MCU::Twi::MCtrlA_t;
            using mctrlb_t = MCU::Twi::MCtrlB_t;
            using mstatus_t = MCU::Twi::MStatus_t;
            
            using size_type = etl::uint_ranged<uint8_t, 0, Size>;
            using index_type = etl::uint_ranged<uint8_t, 0, Size - 1>;
            
            inline static void init() {
                *mcu_twi()->masterBaudrate = baud;
                mcu_twi()->mctrla.template set<mctrla_t::enable>();
                mcu_twi()->mstatus.template reset<mstatus_t::busStateIdle>();
            }
            
            inline static bool startReadByte(const Address adr) {
                if (mState != State::Idle) {
                    return false;
                }
                mBA = BusAddress(adr, Read{});
                mState = State::TransferReadAdr;
                return true;
            }
            
            inline static bool getLastRead(std::byte& out) {
                if (mState != State::TransferReadComplete) {
                    return false;
                }
                out = lastReadData;
                mState = State::Idle;
                return true;
            }
            
            inline static bool write(const Address adr, const std::pair<std::uint8_t, std::uint8_t>& data) {
                if (mState != State::Idle) {
                    return false;
                }
                mData.clear();
                mData.push_back(std::byte{data.first});
                mData.push_back(std::byte{data.second});
                mBA = BusAddress(adr, Write{});
                mState = State::TransferWriteAdr;
                return true;
            }        
            inline static bool write(const Address adr, const std::pair<std::byte, std::byte>& data) {
                if (mState != State::Idle) {
                    return false;
                }
                mData.clear();
                mData.push_back(data.first);
                mData.push_back(data.second);
                mBA = BusAddress(adr, Write{});
                mState = State::TransferWriteAdr;
                return true;
            }        
            
            template<auto N>
            inline static bool write(const Address adr, const std::array<std::byte, N>& data) {
                if (mState != State::Idle) {
                    return false;
                }
                mData.clear();
                for(uint8_t i{}; i < data.size(); ++i) {
                    mData.push_back(data[i]);
                }
                mBA = BusAddress(adr, Write{});
                mState = State::TransferWriteAdr;
                return true;
            }        
            template<typename C>
            inline static bool write(const Address adr, const std::byte pre, const C& data) {
                if (mState != State::Idle) {
                    return false;
                }
                mData.clear();
                mData.push_back(pre);
                for(const auto& e : data) {
                    mData.push_back(e);
                }
                mBA = BusAddress(adr, Write{});
                mState = State::TransferWriteAdr;
                return true;
            }        
            
            template<typename T, T... Ts>
            inline static bool write(const Address adr, const AVR::Pgm::Array<T, Ts...>& data) {
                if (mState != State::Idle) {
                    return false;
                }
                mData.clear();
                for(uint8_t i{}; i < data.size(); ++i) {
                    mData.push_back(data[i]);
                }
                mBA = BusAddress(adr, Write{});
                mState = State::TransferWriteAdr;
                return true;
            }
            
            inline static bool isIdle() {
                return mState == State::Idle;
            }
            inline static auto state() {
                return mState;
            }
            
            inline static void periodic() {
                const auto oldState = mState;
                switch(mState) {
                case State::Undefined:
                    mState = State::Idle;
                break;
                case State::Idle:
                break;
                case State::TransferReadAdr:
                    *mcu_twi()->masterAddress = mBA.mValue;
                    mState = State::TransferReadWait;
                break;
                case State::TransferReadWait:
                    if (mcu_twi()->mstatus.template isSet<mstatus_t::readIF | mstatus_t::writeIF>()) {
                        if (mcu_twi()->mstatus.template isSet<mstatus_t::rxAck>()) {
                            mState = State::Error;
                        }
                        else {
                            mState = State::TransferReadData;
                        }
                    };
                    mcu_twi()->mstatus.template testAndReset<mstatus_t::busErr | mstatus_t::arbLost>([]{
                        mState = State::Error;
                    });
                break;
                case State::TransferReadData:
                    lastReadData = *mcu_twi()->masterData;
                    mState = State::TransferReadStop;
                    mcu_twi()->mstatus.template testAndReset<mstatus_t::busErr | mstatus_t::arbLost>([]{
                        mState = State::Error;
                    });
                break;
                case State::TransferReadStop:
                    mcu_twi()->mctrlb.template set<mctrlb_t::MStop | mctrlb_t::ackact>();
                    mState = State::TransferReadComplete;
                break;
                case State::TransferReadComplete:
                break;
                case State::TransferWriteAdr:
                    *mcu_twi()->masterAddress = mBA.mValue;
                    mState = State::TransferWriteWait;
                break;
                case State::TransferWriteWait:
                    mcu_twi()->mstatus.template testAndReset<mstatus_t::readIF | mstatus_t::writeIF>([]{
                        if (mcu_twi()->mstatus.template isSet<mstatus_t::rxAck>()) {
                            mState = State::Error;
                        }
                        else {
                            mState = State::TransferWriteData;
                        }
                    });
                    mcu_twi()->mstatus.template testAndReset<mstatus_t::busErr | mstatus_t::arbLost>([]{
                        mState = State::Error;
                    });
                break;
                case State::TransferWriteData:
                    if (const auto b = mData.pop_front(); b) {
                        *mcu_twi()->masterData = *b;
                        mState = State::TransferWriteWait;
                    }
                    else {
                        mState = State::Stop;
                    }
                break;
                case State::Stop:
                    mcu_twi()->mctrlb.template set<mctrlb_t::MStop>();
                    mState = State::Idle;
                break;
                case State::Error:
                    mcu_twi()->mctrlb.template set<mctrlb_t::MStop>();
                    mcu_twi()->mstatus.template reset<mstatus_t::busStateIdle>();
                    mState = State::Idle;
                break;
                }
                if (oldState != mState) {
                    switch(mState) {
                    case State::Undefined:
                    break;
                    case State::Idle:
                    break;
                    case State::TransferReadAdr:
                    break;
                    case State::TransferReadWait:
                    break;
                    case State::TransferReadData:
                    break;
                    case State::TransferReadStop:
                    break;
                    case State::TransferReadComplete:
                    break;
                    case State::TransferWriteAdr:
                    break;
                    case State::TransferWriteWait:
                    break;
                    case State::TransferWriteData:
                    break;
                    case State::Stop:
                    break;
                    case State::Error:
                    break;
                    }
                }
            }
            //            inline static auto data_pop() {
            //                return mData.pop_front();
            //            }
        private:
            inline static BusAddress mBA;
            inline static etl::FiFo<std::byte, Size> mData;
            inline static std::byte lastReadData{};
            inline static State mState{State::Undefined};
            
            inline static constexpr uint8_t baud = []{
                constexpr External::Units::hertz fTwi{100'000};
                constexpr uint16_t b = Project::Config::fMcu / (2 * fTwi) - 5;
                static_assert(b <= 255);
                return b;
            }();
        };
    }
}
