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

#include <cstdint>
#include "util/disable.h"
#include "units/duration.h"
#include "sumdprotocoll.h"

namespace Hott {
    
    struct MultiChannel {
        inline static constexpr uint8_t size = 8;
        enum class State {Off = 0, Up, Down};

        uint16_t mData{0};
        
        State state(uint8_t index) volatile {
            assert(index < size);
            uint16_t mask = 0b11 << (2 * index);
            return (State)((mData & mask) >> (2 * index));
        }
        void set(uint8_t index, uint8_t fromValue) volatile {
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
    };
    
    template<uint8_t M>
    class SumDProtocollAdapter final {
        enum sumdstate {Undefined = 0, Start1, StartNormal, StartFailSafe, ChannelDataL, ChannelDataH, CrcL, CrcH, NumberOfStates};
        typedef enum sumdstate sumdstate_t;
        
        enum class MultiState : uint8_t {Undefined = 0, Sync1, Sync2, Data};
        inline static MultiState mMultiState = MultiState::Undefined;
        inline static uint8_t mMultiChannel = 0;
        
        inline static uint8_t mChannelForMultiChannel = 7;
        
        template<int N, typename PA> friend class AVR::Usart;
        
    public:
        SumDProtocollAdapter() = delete;
        
        static uint_ranged<uint16_t, Hott::SumDMsg::ExtendedLow, Hott::SumDMsg::ExtendedHigh> valueExtended(uint8_t channel) {
            Scoped<DisbaleInterrupt<>> di;
            return std::combinedValue(mMsg.channelData[channel]);
        }

        static uint_ranged<uint16_t, Hott::SumDMsg::Low, Hott::SumDMsg::High> value(uint8_t channel) {
            Scoped<DisbaleInterrupt<>> di;
            auto v = std::combinedValue(mMsg.channelData[channel]);
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
        static uint_ranged<uint8_t, Hott::SumDMsg::Low8Bit, Hott::SumDMsg::High8Bit> value8Bit(uint8_t channel) {
            if (mMsg.channelData[channel].first < Hott::SumDMsg::Low8Bit) {
                return Hott::SumDMsg::Low8Bit;
            }
            else if (mMsg.channelData[channel].first > Hott::SumDMsg::High8Bit) {
                return Hott::SumDMsg::High8Bit;
            } 
            else {
                return mMsg.channelData[channel].first;
            }
        }
        static uint_ranged<uint8_t, Hott::SumDMsg::ExtendedLow8Bit, Hott::SumDMsg::ExtendedHigh8Bit> value8BitExtended(uint8_t channel) {
            return mMsg.channelData[channel].first;
        }
        static uint8_t numberOfChannels() {
            return mMsg.nChannels;
        }
        static bool hasMultiChannel() {
            return mHasMultiChannel;
        }
        static MultiChannel::State mChannel(uint8_t channel) {
            return mMultiData.state(channel);
        }

    private:
        inline static bool process(std::byte  c) { // from isr only
            static sumdstate state = sumdstate::Undefined;
            static uint8_t channel = 0;
            
            switch (state) {
            case sumdstate::Undefined:
                if (c == std::byte{0xa8}) {
                    state = sumdstate::Start1;
                }
                else {
                    state = sumdstate::Undefined;
                }
                break;
            case sumdstate::Start1:
                if (c == std::byte{0x01}) {
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
                mMsg.nChannels = std::to_integer<uint8_t>(c);
                state = sumdstate::ChannelDataH;
                break;
            case sumdstate::StartFailSafe:
                mMsg.nChannels = std::to_integer<uint8_t>(c);
                state = sumdstate::ChannelDataH;
                break;
            case sumdstate::ChannelDataH:
                mMsg.channelData[channel].first = std::to_integer<uint8_t>(c);
                state = sumdstate::ChannelDataL;
                break;
            case sumdstate::ChannelDataL:
                mMsg.channelData[channel].second = std::to_integer<uint8_t>(c);
                state = sumdstate::ChannelDataH;
                ++channel;
                if (channel < mMsg.nChannels) {
                    state = sumdstate::ChannelDataH;
                }
                else {
                    state = sumdstate::CrcH;
                    channel = 0;
                }
                if (channel >= SumDMsg::MaxChannels) {
                    channel = 0;
                }
                break;
            case sumdstate::CrcH:
                mMsg.crc = std::to_integer<uint8_t>(c) << 8;
                state = sumdstate::CrcL;
                break;
            case sumdstate::CrcL:
                mMsg.crc |= std::to_integer<uint8_t>(c);
                state = sumdstate::Undefined;
                
                switch(mMultiState) {
                case MultiState::Undefined:
                    if (value8BitExtended(mChannelForMultiChannel) == Hott::SumDMsg::ExtendedHigh8Bit) {
                        mMultiState = MultiState::Sync1;
                    }
                    break;
                case MultiState::Sync1:
                    if (value8BitExtended(mChannelForMultiChannel) == Hott::SumDMsg::ExtendedHigh8Bit ) {
                        mMultiState = MultiState::Sync2;
                    }
                    else {
                        mMultiState = MultiState::Undefined;
                    }
                    break;
                case MultiState::Sync2:
                    if (value8BitExtended(mChannelForMultiChannel) < Hott::SumDMsg::ExtendedHigh8Bit) {
                        mHasMultiChannel = true;
                        mMultiState = MultiState::Data;
                        mMultiChannel = 0;
                        mMultiData.set(mMultiChannel++, value8Bit(mChannelForMultiChannel));
                    }
                    break;
                case MultiState::Data:
                    if (value8BitExtended(mChannelForMultiChannel) == Hott::SumDMsg::ExtendedHigh8Bit) {
                        mMultiState = MultiState::Sync1;
                    }
                    else {
                        if (mMultiChannel < mMultiData.size) {
                            mMultiData.set(mMultiChannel++, value8Bit(mChannelForMultiChannel));
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
                
                break;
            default:
                assert(false);
                break;
            }
            return true;
        }
        
        inline static volatile SumDMsg mMsg;
        inline static volatile MultiChannel mMultiData;
        inline static bool mHasMultiChannel = false;
    };
    
}