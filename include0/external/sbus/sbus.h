#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <chrono>

#include <etl/ranged.h>
#include <etl/algorithm.h>

#include <mcu/internals/usart.h>

namespace External {
    namespace SBus {
#if 0
#define RC_CHANNEL_MIN 990
#define RC_CHANNEL_MAX 2010

#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0f
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04
#define SBUS_UPDATE_RATE 15 //ms

void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe){

    static int output[SBUS_CHANNEL_NUMBER] = {0};

    /*
     * Map 1000-2000 with middle at 1500 chanel values to
     * 173-1811 with middle at 992 S.BUS protocol requires
     */
    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
        output[i] = map(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
    }

    uint8_t stateByte = 0x00;
    if (isSignalLoss) {
        stateByte |= SBUS_STATE_SIGNALLOSS;
    }
    if (isFailsafe) {
        stateByte |= SBUS_STATE_FAILSAFE;
    }
    packet[0] = SBUS_FRAME_HEADER; //Header

    packet[1] = (uint8_t) (output[0] & 0x07FF);
    usart::put((uint8_t) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3);
    packet[3] = (uint8_t) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6);
    packet[4] = (uint8_t) ((output[2] & 0x07FF)>>2);
    packet[5] = (uint8_t) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1);
    packet[6] = (uint8_t) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4);
    packet[7] = (uint8_t) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7);
    packet[8] = (uint8_t) ((output[5] & 0x07FF)>>1);
    packet[9] = (uint8_t) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2);
    packet[10] = (uint8_t) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5);
    packet[11] = (uint8_t) ((output[7] & 0x07FF)>>3);
    packet[12] = (uint8_t) ((output[8] & 0x07FF));
    packet[13] = (uint8_t) ((output[8] & 0x07FF)>>8 | (output[9] & 0x07FF)<<3);
    packet[14] = (uint8_t) ((output[9] & 0x07FF)>>5 | (output[10] & 0x07FF)<<6);  
    packet[15] = (uint8_t) ((output[10] & 0x07FF)>>2);
    packet[16] = (uint8_t) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1);
    packet[17] = (uint8_t) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4);
    packet[18] = (uint8_t) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7);
    packet[19] = (uint8_t) ((output[13] & 0x07FF)>>1);
    packet[20] = (uint8_t) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2);
    packet[21] = (uint8_t) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5);
    packet[22] = (uint8_t) ((output[15] & 0x07FF)>>3);

    packet[23] = stateByte; //Flags byte
    packet[24] = SBUS_FRAME_FOOTER; //Footer
}

uint8_t sbusPacket[SBUS_PACKET_LENGTH];
int rcChannels[SBUS_CHANNEL_NUMBER];
uint32_t sbusTime = 0;

#endif   
        namespace Servo {
            using namespace std::literals::chrono;
            using namespace External::Units::literals;
            
            template<auto N, typename Timer, typename Dbg = void>
            struct ProtocollAdapter {
                enum class State : uint8_t {Undefined, Data, GotEnd, WaitEnd};

                using data_t = std::array<uint16_t, 16>; 
                using value_type = etl::uint_ranged_NaN<uint16_t, 172, 1810>;
                using mapped_type = etl::uint_ranged_NaN<uint16_t, 480, 1504>;
                using channel_t = etl::uint_ranged_NaN<uint8_t, 0, data_t::size() - 1>;
                
                static inline constexpr std::byte start_byte = 0x0f_B;
                static inline constexpr std::byte end_byte = 0x00_B;
                
                static inline value_type value(const channel_t ch) {
                    if (const uint8_t chi = ch.toInt(); ch) {
                        return mChannels[chi];
                    }
                    return value_type{};
                }
                static inline mapped_type valueMapped(const channel_t ch) {
                    if (const uint8_t chi = ch.toInt(); ch) {
                        return mChannels[chi];
                    }
                    return mapped_type{};
                }

                static_assert(Timer::frequency >= 1000_Hz);
                
                static inline uint8_t tc{0};
                static inline void ratePeriodic() {
                    if (++tc > 5) {
                        mState = State::Undefined;
                    }
                }
                
                static inline bool process(const std::byte b) {
//                    ++c;
                    tc = 0;
                    switch(mState) {
                    case State::Undefined:
                        if (b == end_byte) {
                            mState = State::GotEnd;
                        }
                        else if (b == start_byte) {
                            mState = State::Data;
                            mIndex.setToBottom();
                        }
                        break;
                    case State::GotEnd:
                        if (b == start_byte) {
                            mState = State::Data;
                            mIndex.setToBottom();
                        }
                        else if (b == end_byte) {
                            mState = State::GotEnd;
                        }
                        else {
                            mState = State::Undefined;
                        }
                        break;
                    case State::Data:
                        mData[mIndex] = std::to_integer(b);
                        if (mIndex.isTop()) {
                            mState = State::WaitEnd;
                        }
                        else {
                            ++mIndex;
                        }
                        break;
                    case State::WaitEnd:
                        if (b == end_byte) {
                            mState = State::GotEnd;
                            decode();
                        }
                        else {
                            mState = State::Undefined;
                        }
                        break;
                    }
                    return true;
                }
                
//                inline static uint8_t c{};
                
//            private:
                static inline void decode() {
                    mChannels[0]  = (uint16_t) (((mData[0]    | mData[1] << 8))                     & 0x07FF);
                    mChannels[1]  = (uint16_t) ((mData[1]>>3  | mData[2] <<5)                     & 0x07FF);
                    mChannels[2]  = (uint16_t) ((mData[2]>>6  | mData[3] <<2 |mData[4]<<10)  	 & 0x07FF);
                    mChannels[3]  = (uint16_t) ((mData[4]>>1  | mData[5] <<7)                     & 0x07FF);
                    mChannels[4]  = (uint16_t) ((mData[5]>>4  | mData[6] <<4)                     & 0x07FF);
                    mChannels[5]  = (uint16_t) ((mData[6]>>7  | mData[7] <<1 |mData[8]<<9)   	 & 0x07FF);
                    mChannels[6]  = (uint16_t) ((mData[8]>>2  | mData[9] <<6)                     & 0x07FF);
                    mChannels[7]  = (uint16_t) ((mData[9]>>5  | mData[10]<<3)                     & 0x07FF);
                    mChannels[8]  = (uint16_t) ((mData[11]    | mData[12]<<8)                     & 0x07FF);
                    mChannels[9]  = (uint16_t) ((mData[12]>>3 | mData[13]<<5)                     & 0x07FF);
                    mChannels[10] = (uint16_t) ((mData[13]>>6 | mData[14]<<2 |mData[15]<<10) 	 & 0x07FF);
                    mChannels[11] = (uint16_t) ((mData[15]>>1 | mData[16]<<7)                     & 0x07FF);
                    mChannels[12] = (uint16_t) ((mData[16]>>4 | mData[17]<<4)                     & 0x07FF);
                    mChannels[13] = (uint16_t) ((mData[17]>>7 | mData[18]<<1 |mData[19]<<9)  	 & 0x07FF);
                    mChannels[14] = (uint16_t) ((mData[19]>>2 | mData[20]<<6)                     & 0x07FF);
                    mChannels[15] = (uint16_t) ((mData[20]>>5 | mData[21]<<3)                     & 0x07FF);
                }
                using MesgType = std::array<uint8_t, 23>;

                inline static data_t mChannels;
                
                inline static State mState{State::Undefined};
                inline static MesgType mData; 
                inline static etl::index_type_t<MesgType> mIndex;
            };
        }
        namespace Output {
            template<typename CN>
            struct Generator {
                
                using usart = AVR::Usart<CN, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<2>, AVR::SendQueueLength<128>>;
                
                inline static constexpr uint16_t sbus_min = 172;
                inline static constexpr uint16_t sbus_max = 1811;
                
                inline static constexpr uint16_t sbus_mid = (sbus_max + sbus_min) / 2;
                
                inline static void init() {
                    usart::template init<AVR::BaudRate<100000>, AVR::FullDuplex, true, 1>();
                    for(auto& o : output) {
                        o = (sbus_max + sbus_min) / 2;
                    }
                }

                static inline constexpr std::byte sbus_start = 0x0f_B;

                inline static void periodic() {
                    usart::periodic();
                }
                
                inline static void ratePeriodic() { // 14ms
                    usart::put(sbus_start);
                
                    usart::put((std::byte) (output[0] & 0x07FF));
                    usart::put((std::byte) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3));
                    usart::put((std::byte) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6));
                    usart::put((std::byte) ((output[2] & 0x07FF)>>2));
                    usart::put((std::byte) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1));
                    usart::put((std::byte) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4));
                    usart::put((std::byte) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7));
                    usart::put((std::byte) ((output[5] & 0x07FF)>>1));
                    usart::put((std::byte) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2));
                    usart::put((std::byte) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5));
                    usart::put((std::byte) ((output[7] & 0x07FF)>>3));
                    usart::put((std::byte) ((output[8] & 0x07FF)));
                    usart::put((std::byte) ((output[8] & 0x07FF)>>8 | (output[9] & 0x07FF)<<3));
                    usart::put((std::byte) ((output[9] & 0x07FF)>>5 | (output[10] & 0x07FF)<<6));  
                    usart::put((std::byte) ((output[10] & 0x07FF)>>2));
                    usart::put((std::byte) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1));
                    usart::put((std::byte) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4));
                    usart::put((std::byte) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7));
                    usart::put((std::byte) ((output[13] & 0x07FF)>>1));
                    usart::put((std::byte) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2));
                    usart::put((std::byte) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5));
                    usart::put((std::byte) ((output[15] & 0x07FF)>>3));
                
                    usart::put(0x00_B); //Flags byte
                    usart::put(0x00_B); //Footer
                }
//            private:
                static inline std::array<uint16_t, 16> output;                 
            };  
        }
    }
}
