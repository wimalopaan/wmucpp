#pragma once

#include "sumdprotocoll.h"

#include <external/solutions/tick.h>
#include <etl/algorithm.h>

namespace Hott {
    namespace Output {
        using namespace std::literals::chrono;
        
        template<typename CN, typename Timer>
        struct GeneratorV1 {
            static constexpr External::Tick<Timer> timeoutTicks{10_ms};
            static_assert(timeoutTicks.value > 1);
//                std::integral_constant<uint8_t, timeoutTicks.value>::_;
            
            using usart = AVR::Usart<CN, External::Hal::NullProtocollAdapter<>, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<2>, AVR::SendQueueLength<128>>;
            
            inline static constexpr uint16_t sbus_min = 172;
            inline static constexpr uint16_t sbus_max = 1811;
            
            inline static constexpr uint16_t sbus_mid = (sbus_max + sbus_min) / 2;
            
            using value_type = etl::uint_ranged<uint16_t, sbus_min, sbus_max>;
            using index_type = etl::uint_ranged<uint8_t, 0, 15>;
            
            inline static void init() {
                usart::template init<AVR::BaudRate<115200>>();
                for(auto& o : output) {
                    o = (sbus_max + sbus_min) / 2;
                }
            }

            static inline void set(const index_type& i, const value_type& v) {
                output[i] = v;
            }
            
            static inline constexpr std::byte sbus_start = 0x0f_B;

            inline static void periodic() {
                usart::periodic();
            }
            
            inline static void ratePeriodic() { // 14ms
                ++ticks;
                ticks.on(timeoutTicks, []{
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
                    
                });
            }
        private:
            static inline std::array<uint16_t, 16> output;                 
            static inline External::Tick<Timer> ticks{};
        };  

        template<typename CN, typename Timer, typename dbg = void, typename PA = External::Hal::NullProtocollAdapter<>, uint8_t Size = 128>
        struct GeneratorV3 {
            static constexpr External::Tick<Timer> timeoutTicks{10_ms};
            static_assert(timeoutTicks.value > 1);
//                std::integral_constant<uint8_t, timeoutTicks.value>::_;
            
//            using usart = AVR::Usart<CN, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<2>, AVR::SendQueueLength<128>>;
            using usart = std::conditional_t<std::is_same_v<PA, External::Hal::NullProtocollAdapter<>>, 
                                           AVR::Usart<CN, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<2>, AVR::SendQueueLength<Size>>,
                                           AVR::Usart<CN, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<Size>>
                                           >;
            
            inline static constexpr uint16_t sumd_min = Hott::SumDMsgV3::Low;
            inline static constexpr uint16_t sumd_max = Hott::SumDMsgV3::High;
            inline static constexpr uint16_t sumd_mid = Hott::SumDMsgV3::Mid;
            
            static inline constexpr uint8_t numberOfWords = 18;
            static inline constexpr uint8_t maxChannels = 32;
            static inline constexpr uint8_t numberOfSwitches = 64 / 8;
            
            using value_type = etl::uint_ranged<uint16_t, sumd_min, sumd_max>;
            using index_type = etl::uint_ranged<uint8_t, 0, 31>;
            using switch_index_type = etl::uint_ranged<uint8_t, 0, 63>;
            using command_t = std::pair<std::byte, std::byte>;
            
            enum class State : uint8_t {Ch1to16 = 0x02, Ch1to8and17to24 = 0x03, Ch1to8and25to32 = 0x04, ch1to8andSwitches = 0x05};
            
            inline static void init() {
                usart::template init<AVR::BaudRate<115200>>();
                for(auto& o : channels) {
                    o = sumd_mid;
                }
            }

            static inline void setCmd(const command_t& c) {
                modeCmd = c.first;
                subCmd = c.second;
            }
            
            static inline void setSwitch(const switch_index_type& i, const bool v) {
                const uint8_t byteNumber = 7 - (i / 8);
                const uint8_t bitNumber = i & 0x07;
                const std::byte mask = (0x01_B << bitNumber);
                if (v) {
                    switches[byteNumber] |= mask;
                }
                else {
                    switches[byteNumber] &= ~mask;
                }
            }
            
            static inline void set(const index_type& i, const value_type& v) {
                channels[i] = v;
            }

            static inline uint16_t& raw(const index_type& i) {
                return channels[i];
            }
            
            inline static void periodic() {
                usart::periodic();
            }
            
            inline static void ratePeriodic() { // 14ms
                ++ticks;
                ticks.on(timeoutTicks, []{
                    csum = 0;
                    usart::put(Hott::SumDMsgV3::start_code);
                    etl::crc16(csum, (uint8_t)Hott::SumDMsgV3::start_code);
                    usart::put(Hott::SumDMsgV3::version_code);
                    etl::crc16(csum, (uint8_t)Hott::SumDMsgV3::version_code);
                    usart::put(std::byte(numberOfWords));
                    etl::crc16(csum, numberOfWords);
                    
                    switch(mState) {
                    case State::Ch1to16:
                        putChannels<0, 15>();
                        putFunctionCode();
                        mState = State::Ch1to8and17to24;
                        break;
                    case State::Ch1to8and17to24:
                        putChannels<0, 7>();
                        putChannels<16, 23>();
                        putFunctionCode();
                        mState = State::Ch1to8and25to32;
                        break;
                    case State::Ch1to8and25to32:
                        putChannels<0, 7>();
                        putChannels<24, 31>();
                        putFunctionCode();
                        mState = State::ch1to8andSwitches;
                        break;
                    case State::ch1to8andSwitches:
                        putChannels<0, 11>();
                        putSwitches();
                        putFunctionCode();
                        mState = State::Ch1to16;
                        break;
                    }
                    putCmds();
                    putCsum();
                });
            }
        private:
            template<uint8_t From, uint8_t To>
            inline static void putChannels() {
                for(uint8_t i = From; i <= To; ++i) {
                    usart::put(etl::nth_byte<1>(channels[i]));
                    etl::crc16(csum, (uint8_t)etl::nth_byte<1>(channels[i]));
                    usart::put(etl::nth_byte<0>(channels[i]));
                    etl::crc16(csum, (uint8_t)etl::nth_byte<0>(channels[i]));
                }
            }
            static inline void putSwitches() {
                for(uint8_t i = 0; i < switches.size(); ++i) {
                    usart::put(switches[i]);
                    etl::crc16(csum, (uint8_t)switches[i]);
                }
            }
            static inline void putFunctionCode() {
                usart::put(std::byte(mState));
                etl::crc16(csum, uint8_t(mState));
            }
            static inline void putCmds() {
                usart::put(reserved);
                etl::crc16(csum, uint8_t(reserved));
                usart::put(modeCmd);
                etl::crc16(csum, uint8_t(modeCmd));
                usart::put(subCmd);
                etl::crc16(csum, uint8_t(subCmd));
            }
            static inline void putCsum() {
                usart::put(etl::nth_byte<1>(csum));
                usart::put(etl::nth_byte<0>(csum));                
            }

            static inline uint16_t csum = 0;
            static inline State mState{State::Ch1to16};
            static inline std::byte reserved{};
            static inline std::byte modeCmd{};
            static inline std::byte subCmd{};
            static inline std::array<uint16_t, maxChannels> channels;                 
            static inline std::array<std::byte, numberOfSwitches> switches;                 
            static inline External::Tick<Timer> ticks{};
        };  
    }
}
