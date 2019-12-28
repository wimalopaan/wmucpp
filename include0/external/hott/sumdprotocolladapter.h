/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2019 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include "sumdprotocoll.h"

namespace Hott {
    struct MultiChannel final {
        inline static constexpr uint8_t size = 8;
        enum class State {Off = 0, Up, Down};
        inline State state(uint8_t index) const {
            assert(index < size);
            uint16_t mask = 0b11 << (2 * index);
            return (State)((mData & mask) >> (2 * index));
        }
        inline void set(uint8_t index, uint8_t fromValue) {
            uint16_t mask = 0b11 << (2 * index);
            if (fromValue == SumDMsg::High8Bit) {
                mData = (mData & ~mask) | ((uint16_t)State::Up << (2 * index));
            }
            else if (fromValue == SumDMsg::Mid8Bit) {
                mData = (mData & ~mask) | ((uint16_t)State::Off << (2 * index));
            }
            else if (fromValue == SumDMsg::Low8Bit) {
                mData = (mData & ~mask) | ((uint16_t)State::Down << (2 * index));
            }
        }
    private:
        uint16_t mData{0};
    };
    
    template<uint8_t M, etl::Concepts::NamedFlag UseInts = AVR::UseInterrupts<true>>
    class SumDProtocollAdapter final {
        enum sumdstate {Undefined = 0, Start1, StartNormal, StartFailSafe, ChannelDataL, ChannelDataH, CrcL, CrcH, NumberOfStates};
        typedef enum sumdstate sumdstate_t;
        
        enum class MultiState : uint8_t {Undefined = 0, Sync1, Sync2, Data};

        using MultiStateType = std::conditional_t<UseInts::value, volatile MultiState, MultiState>;
        inline static MultiStateType mMultiState = MultiState::Undefined;
        inline static uint8_t mMultiChannel = 0;

        using channelForMutliChannelType = std::conditional_t<UseInts::value, volatile uint8_t, uint8_t>;
        inline static channelForMutliChannelType mChannelForMultiChannel = 7;

        using sumDMesgType = std::conditional_t<UseInts::value, volatile SumDMsg, SumDMsg>;
        using multiCHType = std::conditional_t<UseInts::value, volatile MultiChannel, MultiChannel>;
        using hasMultiChannelType = std::conditional_t<UseInts::value, volatile bool, bool>;
        using validType = std::conditional_t<UseInts::value, volatile bool, bool>;
        using counterType = std::conditional_t<UseInts::value, volatile uint16_t, uint16_t>;
        
    public:
//        using value_type = etl::uint_ranged_NaN<uint16_t, Hott::SumDMsg::Low, Hott::SumDMsg::High>;
        using value_type = Hott::hott_t;

        SumDProtocollAdapter() = delete;
        
        inline static etl::uint_ranged<uint16_t, Hott::SumDMsg::ExtendedLow, Hott::SumDMsg::ExtendedHigh> valueExtended(uint8_t channel) {
            etl::Scoped<etl::DisbaleInterrupt<>, UseInts::value> di;
            if (!mMsgInactive->valid) {
                return {};
            }
            uint16_t v = etl::combinedValue(mMsgInactive->channelData[channel]);
            if (v < Hott::SumDMsg::ExtendedLow) {
                return {Hott::SumDMsg::ExtendedLow};
            }
            else if (v > Hott::SumDMsg::ExtendedHigh) {
                return {Hott::SumDMsg::ExtendedHigh};
            }
            else {
                return {v};
            }
        }
        
        inline static etl::uint_ranged_NaN<uint16_t, Hott::SumDMsg::Low, Hott::SumDMsg::High> value(uint8_t channel) {
            etl::Scoped<etl::DisbaleInterrupt<>, UseInts::value> di;

            if (!mMsgInactive->valid) {
                return {};
            }
            uint16_t v = etl::combinedValue(mMsgInactive->channelData[channel]);
            if (v < Hott::SumDMsg::Low) {
                return Hott::SumDMsg::Low;
            }
            else if (v > Hott::SumDMsg::High) {
                return Hott::SumDMsg::High;
            }
            else {
                return v;
            }
        }
        inline static etl::uint_ranged<uint8_t, Hott::SumDMsg::Low8Bit, Hott::SumDMsg::High8Bit> value8Bit(uint8_t channel) {
            etl::Scoped<etl::DisbaleInterrupt<>, UseInts::value> di;

            if (!mMsgInactive->valid) {
                return {};
            }
            return value8Bit_unsafe(channel);
        }
        inline static etl::uint_ranged<uint8_t, Hott::SumDMsg::ExtendedLow8Bit, Hott::SumDMsg::ExtendedHigh8Bit> value8BitExtended(uint8_t channel) {
            etl::Scoped<etl::DisbaleInterrupt<>, UseInts::value> di;

            if (!mMsgInactive->valid) {
                return {};
            }
            return value8BitExtended_unsafe(channel);
        }
        inline static uint8_t numberOfChannels() {
            return mMsgInactive->nChannels;
        }
        inline static bool hasMultiChannel() {
            return mHasMultiChannel;
        }
        inline static MultiChannel::State mChannel(uint8_t channel) {
            return mMultiData.state(channel);
        }
        
        inline static bool process(std::byte  c) { // (ca 3µs)
            static sumdstate state = sumdstate::Undefined;
            static uint8_t channel = 0;
            static uint16_t crc = 0;
            static uint8_t actualHighValue = Hott::SumDMsg::Mid8Bit;        
            static uint8_t actualLowValue  = 0;
            
            switch (state) {
            case sumdstate::Undefined:
                if (c == SumDMsg::start_code) {
                    crc = 0;
                    etl::crc16(crc, std::to_integer(c));
                    state = sumdstate::Start1;
                }
                else {
                    state = sumdstate::Undefined;
                }
                break;
            case sumdstate::Start1:
                if (c == SumDMsg::version_code) {
                    etl::crc16(crc, std::to_integer(c));
                    state = sumdstate::StartNormal;
                }
                else if (c == std::byte{0x81}) {
                    state = sumdstate::StartFailSafe;
                }
                else {
                    state = sumdstate::Undefined;
                }
                break;
            case sumdstate::StartNormal:
                mMsgActive->nChannels = std::to_integer(c);
                mMsgActive->valid = false;
                etl::crc16(crc, mMsgActive->nChannels);
                if (mMsgActive->nChannels < SumDMsg::MaxChannels) {
                    state = sumdstate::ChannelDataH;
                }
                else {
                    state = sumdstate::Undefined;
                }
                break;
            case sumdstate::StartFailSafe:
                mMsgActive->nChannels = std::to_integer(c);
                mMsgActive->valid = false;
                etl::crc16(crc, mMsgActive->nChannels);
                if (mMsgActive->nChannels < SumDMsg::MaxChannels) {
                    state = sumdstate::ChannelDataH;
                }
                else {
                    state = sumdstate::Undefined;
                }
                break;
            case sumdstate::ChannelDataH:
                actualHighValue = std::to_integer<uint8_t>(c);
                etl::crc16(crc, actualHighValue);
//                mMsg.channelData[channel].first = std::to_integer<uint8_t>(c);
//                etl::crc16(crc, mMsg.channelData[channel].first);
                state = sumdstate::ChannelDataL;
                break;
            case sumdstate::ChannelDataL:
                actualLowValue = std::to_integer<uint8_t>(c);
                etl::crc16(crc, actualLowValue);
//                mMsg.channelData[channel].second = std::to_integer<uint8_t>(c);
//                etl::crc16(crc, mMsg.channelData[channel].second);

                mMsgActive->channelData[channel] = {actualHighValue, actualLowValue};
                state = sumdstate::ChannelDataH;
                ++channel;
                if (channel < mMsgActive->nChannels) {
                    state = sumdstate::ChannelDataH;
                }
                else {
                    state = sumdstate::CrcH;
                    channel = 0;
                }
                break;
            case sumdstate::CrcH:
                mMsgActive->crc = std::to_integer(c) << 8;
                state = sumdstate::CrcL;
                break;
            case sumdstate::CrcL:
                ++mPackageCounter;
                mMsgActive->crc |= std::to_integer(c);
                
                state = sumdstate::Undefined;
                
                if (crc == mMsgActive->crc) {
                    mMsgActive->valid = true;
                    std::swap(mMsgActive, mMsgInactive);
                    
                    if (auto vm = value8BitExtended_unsafe(mChannelForMultiChannel)) {
                        
                        switch(mMultiState) {
                        case MultiState::Undefined:
                            if (vm == Hott::SumDMsg::ExtendedHigh8Bit) {
                                mMultiState = MultiState::Sync1;
                            }
                            break;
                        case MultiState::Sync1:
                            if (vm == Hott::SumDMsg::ExtendedHigh8Bit ) {
                                mMultiState = MultiState::Sync2;
                            }
                            else {
                                mMultiState = MultiState::Undefined;
                            }
                            break;
                        case MultiState::Sync2:
                            if (vm < Hott::SumDMsg::ExtendedHigh8Bit) {
                                mHasMultiChannel = true;
                                mMultiState = MultiState::Data;
                                mMultiChannel = 0;
                                mMultiData.set(mMultiChannel++, value8Bit_unsafe(mChannelForMultiChannel));
                            }
                            break;
                        case MultiState::Data:
                            if (vm == Hott::SumDMsg::ExtendedHigh8Bit) {
                                mMultiState = MultiState::Sync1;
                            }
                            else {
                                if (mMultiChannel < mMultiData.size) {
                                    mMultiData.set(mMultiChannel++, value8Bit_unsafe(mChannelForMultiChannel));
                                }
                                else {
                                    mHasMultiChannel = false;
                                    mMultiState = MultiState::Undefined;
                                    mMultiChannel = 0;
                                }
                            }
                            break;
                        default:
                            assert(false);
                        }
                    }
                    
                }
                else {
                    mMultiState = MultiState::Undefined;
                }
                break;
            default:
                assert(false);
                break;
            }
            return true;
        }
        inline static bool valid() {
            return mMsgInactive->valid;
        }
        inline static uint16_t packageCount() {
            return mPackageCounter;
        }
        inline static void resetCount() {
            mPackageCounter = 0;
        }
    private:
        inline static etl::uint_ranged<uint8_t, Hott::SumDMsg::Low8Bit, Hott::SumDMsg::High8Bit> value8Bit_unsafe(uint8_t channel) {
            if (mMsgInactive->channelData[channel].first < Hott::SumDMsg::Low8Bit) {
                return etl::uint_ranged<uint8_t, Hott::SumDMsg::Low8Bit, Hott::SumDMsg::High8Bit>{Hott::SumDMsg::Low8Bit};
            }
            else if (mMsgInactive->channelData[channel].first > Hott::SumDMsg::High8Bit) {
                return etl::uint_ranged<uint8_t, Hott::SumDMsg::Low8Bit, Hott::SumDMsg::High8Bit>{Hott::SumDMsg::High8Bit};
            } 
            else {
                return etl::uint_ranged<uint8_t, Hott::SumDMsg::Low8Bit, Hott::SumDMsg::High8Bit>{mMsgInactive->channelData[channel].first};
            }
        }
        inline static etl::uint_ranged<uint8_t, Hott::SumDMsg::ExtendedLow8Bit, Hott::SumDMsg::ExtendedHigh8Bit> value8BitExtended_unsafe(uint8_t channel) {
            if (mMsgInactive->channelData[channel].first < Hott::SumDMsg::ExtendedLow8Bit) {
                return etl::uint_ranged<uint8_t, Hott::SumDMsg::ExtendedLow8Bit, Hott::SumDMsg::ExtendedHigh8Bit>{Hott::SumDMsg::ExtendedLow8Bit};
            }
            else if (mMsgInactive->channelData[channel].first > Hott::SumDMsg::ExtendedHigh8Bit) {
                return etl::uint_ranged<uint8_t, Hott::SumDMsg::ExtendedLow8Bit, Hott::SumDMsg::ExtendedHigh8Bit>{Hott::SumDMsg::ExtendedHigh8Bit};
            } 
            else {
                return etl::uint_ranged<uint8_t, Hott::SumDMsg::ExtendedLow8Bit, Hott::SumDMsg::ExtendedHigh8Bit>{mMsgInactive->channelData[channel].first};
            }
        }
        inline static sumDMesgType mMsg1;
        inline static sumDMesgType mMsg2;
        inline static sumDMesgType* mMsgActive   = &mMsg1;
        inline static sumDMesgType* mMsgInactive = &mMsg2;
        
        inline static multiCHType mMultiData;
        inline static hasMultiChannelType mHasMultiChannel = false;
        inline static counterType mPackageCounter = 0;
    };
}
