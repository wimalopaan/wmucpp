#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <numbers>
#include <chrono>

#include "../include/rc.h"
#include "../include/crc.h"


namespace RC {
    namespace Protokoll {
        namespace Crsf {
            namespace Address {
                inline static constexpr std::byte Broadcast{0x00};
                inline static constexpr std::byte Controller{0xc8};
                inline static constexpr std::byte Handset{0xea};
                inline static constexpr std::byte RX{0xec};
                inline static constexpr std::byte TX{0xee};
            }
            namespace Type {
                inline static constexpr std::byte Link{0x14};
                inline static constexpr std::byte Channels{0x16};
                inline static constexpr std::byte Ping{0x28};
                inline static constexpr std::byte Info{0x29};
                inline static constexpr std::byte ParamEntry{0x2b};
                inline static constexpr std::byte ParamRead{0x2c};
                inline static constexpr std::byte ParamWrite{0x2d};
                inline static constexpr std::byte Command{0x32};
                                
            }
            namespace CommandType {
                inline static constexpr std::byte Crsf{0x10};
                inline static constexpr std::byte CC{0xa0}; // CruiseController
            }
            namespace CrsfCommand {
                inline static constexpr std::byte SelectID{0x05};
            }
            namespace CCCommand {
                inline static constexpr std::byte SetSwitches{0x01};
            }
//            Every frame has the structure:
//            <Device address><Frame length><Type><Payload><CRC>            
            using MesgType = std::array<uint8_t, 64>;
            
            static constexpr uint8_t  ValueBits = 11;
            static constexpr uint16_t ValueMask = ((1 << ValueBits) - 1);
            
            static constexpr uint16_t CenterValue = 0x3e0;            
            
            template<uint8_t N, typename Debug = void, typename MCU = DefaultMcu>
            struct Adapter {
                using MesgType = RC::Protokoll::Crsf::MesgType;
                
                static constexpr uint8_t mPauseCount{2}; // 2ms
                
                static constexpr uint8_t CRSFChannels{16};
                
                enum class State : uint8_t {Undefined, 
                                            GotAddress, GotLength, 
                                            Channels, Data, Command, AwaitCRC, AwaitCRCAndDecode};
                
                static inline void tick1ms() {
                    if (mPauseCounter > 0) {
                        --mPauseCounter;
                    }
                    else {
                        mState = State::Undefined;
                    }
                }
                
                inline static void process(const std::byte b) {
                    mPauseCounter = mPauseCount;
                    ++mBytesCounter;
                    switch(mState) { // enum-switch -> no default (intentional)
                    case State::Undefined:
                        csum.reset();
                        if (b == Crsf::Address::Controller) {
                            mState = State::GotAddress;
                        }
                    break;
                    case State::GotAddress:
                        if ((static_cast<uint8_t>(b) > 2) && (static_cast<uint8_t>(b) <= mData.size())) {
                            mLength = static_cast<uint8_t>(b) - 2; // only payload (not including type and crc)
                            mIndex = 0;
                            mState = State::GotLength;
                        }
                        else {
                            mState = State::Undefined;
                        }
                    break;
                    case State::GotLength:
                        csum += b;
                        if ((b == Crsf::Type::Channels) && (mLength == 22)) {
                            mState = State::Channels;
                        }
                        else if (b == Crsf::Type::Command) {
                            mState = State::Command;
                        }
                        else {
                            mState = State::Data;
                        }
                    break;
                    case State::Command:
                        csum += b;
                        if (++mIndex >= mLength) {
                            mState = State::AwaitCRC;
                        }
                    break;
                    case State::Data:
                        csum += b;
                        if (++mIndex >= mLength) {
                            mState = State::AwaitCRC;
                        }
                    break;
                    case State::Channels:
                        csum += b;
                        mData[mIndex] = static_cast<uint8_t>(b);
                        if (++mIndex >= mLength) {
                            mState = State::AwaitCRCAndDecode;
                        }
                    break;
                    case State::AwaitCRC:
                        if (csum == b) {
                            ++mPackagesCounter;
                        } 
                        mState = State::Undefined;
                    break;
                    case State::AwaitCRCAndDecode:
                        if (csum == b) {
                            ++mPackagesCounter;
                            convert();
                        } 
                        mState = State::Undefined;
                    break;
                    }            
                }        
            private:
                static inline void convert() {
                    mChannels[0]  = (uint16_t) (((mData[0]    | mData[1] << 8))                 & Crsf::ValueMask);
                    mChannels[1]  = (uint16_t) ((mData[1]>>3  | mData[2] <<5)                   & Crsf::ValueMask);
                    mChannels[2]  = (uint16_t) ((mData[2]>>6  | mData[3] <<2 | mData[4]<<10)  	 & Crsf::ValueMask);
                    mChannels[3]  = (uint16_t) ((mData[4]>>1  | mData[5] <<7)                   & Crsf::ValueMask);
                    mChannels[4]  = (uint16_t) ((mData[5]>>4  | mData[6] <<4)                   & Crsf::ValueMask);
                    mChannels[5]  = (uint16_t) ((mData[6]>>7  | mData[7] <<1 | mData[8]<<9)   	 & Crsf::ValueMask);
                    mChannels[6]  = (uint16_t) ((mData[8]>>2  | mData[9] <<6)                   & Crsf::ValueMask);
                    mChannels[7]  = (uint16_t) ((mData[9]>>5  | mData[10]<<3)                   & Crsf::ValueMask);
                    mChannels[8]  = (uint16_t) ((mData[11]    | mData[12]<<8)                   & Crsf::ValueMask);
                    mChannels[9]  = (uint16_t) ((mData[12]>>3 | mData[13]<<5)                   & Crsf::ValueMask);
                    mChannels[10] = (uint16_t) ((mData[13]>>6 | mData[14]<<2 | mData[15]<<10) 	 & Crsf::ValueMask);
                    mChannels[11] = (uint16_t) ((mData[15]>>1 | mData[16]<<7)                   & Crsf::ValueMask);
                    mChannels[12] = (uint16_t) ((mData[16]>>4 | mData[17]<<4)                   & Crsf::ValueMask);
                    mChannels[13] = (uint16_t) ((mData[17]>>7 | mData[18]<<1 | mData[19]<<9)  	 & Crsf::ValueMask);
                    mChannels[14] = (uint16_t) ((mData[19]>>2 | mData[20]<<6)                   & Crsf::ValueMask);
                    mChannels[15] = (uint16_t) ((mData[20]>>5 | mData[21]<<3)                   & Crsf::ValueMask);
                }
                static inline uint16_t packages() {
                    return mPackagesCounter;
                }
                static inline uint16_t getBytes() {
                    return mBytesCounter;
                }
            private:
                inline static CRC8 csum;
                inline static State mState;
                inline static MesgType mData; 
                inline static uint8_t mIndex{};
                inline static uint8_t mLength{};
                inline static uint16_t mPackagesCounter{};
                inline static uint16_t mBytesCounter{};
                inline static uint8_t mPauseCounter{};
                inline static std::array<uint16_t, 16> mChannels;
            };
        }
    }
}
