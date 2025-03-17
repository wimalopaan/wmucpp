#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <chrono>

#include <etl/ranged.h>
#include <etl/algorithm.h>

#include <external/solutions/tick.h>

#include <mcu/internals/usart.h>

namespace External {
    namespace SBus {
        static inline constexpr std::byte start_byte = 0x0f_B;
        static inline constexpr std::byte end_byte = 0x00_B;
        
        static inline constexpr std::byte ch17 = 0x01_B;
        static inline constexpr std::byte ch18 = 0x02_B;
        static inline constexpr std::byte frameLost = 0x04_B;
        static inline constexpr std::byte failSafe = 0x08_B;

        namespace Servo {
            using namespace std::literals::chrono;
            using namespace External::Units::literals;
            
            template<auto N, typename Timer, typename Dbg = void, typename Callback = void>
            struct ProtocollAdapter {
                enum class State : uint8_t {Undefined, Data, GotEnd, WaitEnd};

                using data_t = std::array<uint16_t, 16>; 
                using value_type = etl::uint_ranged_NaN<uint16_t, 172, 1811>;
                using mapped_type = etl::uint_ranged_NaN<uint16_t, 480, 1504>;
                using channel_t = etl::uint_ranged_NaN<uint8_t, 0, data_t::size() - 1>;
                
                static inline std::byte switches() {
                    return mFlagsAndSwitches & (ch17 | ch18);
                }
                
                static inline std::byte flags() {
                    return mFlagsAndSwitches & (failSafe | frameLost);
                }
                
                static inline value_type value(const channel_t ch) {
                    if (const uint8_t chi = ch.toInt(); ch) {
                        if (mValid) {
                            return mChannels[chi];
                        }
                    }
                    return value_type{};
                }
                static inline mapped_type valueMapped(const channel_t ch) {
                    if (const uint8_t chi = ch.toInt(); ch) {
                        if (mValid) {
                            return mChannels[chi];
                        }
                    }
                    return mapped_type{};
                }

                static_assert(Timer::frequency >= 1000_Hz);
                
                static inline constexpr External::Tick<Timer> byteTimeout{2_ms};
                static inline constexpr External::Tick<Timer> packageTimeout{500_ms};

                static inline void ratePeriodic() {
                    ++mByteTimer;
                    ++mPackageTimer;

                    mByteTimer.on(byteTimeout, []{
                        mState = State::Undefined;
                    });                  

                    mPackageTimer.on(packageTimeout, []{
                        mValid = false;
                    });
                }
                
                static inline bool process(const std::byte b) {
                    mByteTimer.reset();
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
                            if constexpr (std::is_same_v<Callback, void>) {
                                decode();
                            }
                            else {
                                Callback::decode(mData);
                            }
                            ++mPackages;
                        }
                        else {
                            mState = State::Undefined;
                        }
                        break;
                    }
                    return true;
                }
                inline static uint16_t packages() {
                    return mPackages;
                }
                inline static void resetStats() {
                    mPackages = 0;
                }
            private:
                static inline External::Tick<Timer> mPackageTimer{};
                static inline External::Tick<Timer> mByteTimer{};

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
                    mFlagsAndSwitches = std::byte(mData[22] & 0x0f);
                    mPackageTimer.reset();
                    mValid = true;
                }

                using MesgType = std::array<uint8_t, 23>;
                inline static data_t mChannels;
                inline static State mState{State::Undefined};
                inline static MesgType mData; 
                inline static etl::index_type_t<MesgType> mIndex;
                inline static uint16_t mPackages{};
                inline static bool mValid{false};
                inline static std::byte mFlagsAndSwitches{0};
            };
        }
        namespace Output {
            using namespace std::literals::chrono;
            
            template<typename CN, typename Timer, typename dbg = void, typename PA = External::Hal::NullProtocollAdapter<>, uint8_t Size = 128>
            struct Generator {
                static constexpr External::Tick<Timer> timeoutTicks{14_ms};
                static_assert(timeoutTicks.value > 1);
//                std::integral_constant<uint8_t, timeoutTicks.value>::_;
                
                using usart = std::conditional_t<std::is_same_v<PA, External::Hal::NullProtocollAdapter<>>, 
                                               AVR::Usart<CN, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<2>, AVR::SendQueueLength<Size>>,
                                               AVR::Usart<CN, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<Size>>
                                               >;
                
                using pa_t = PA;
                
                inline static constexpr uint16_t sbus_min = 172;
                inline static constexpr uint16_t sbus_max = 1811;
                
                inline static constexpr uint16_t sbus_mid = (sbus_max + sbus_min) / 2;
                
                using value_type = etl::uint_ranged<uint16_t, sbus_min, sbus_max>;
                using index_type = etl::uint_ranged<uint8_t, 0, 15>;
                
                inline static void init() {
                    usart::template init<AVR::BaudRate<100000>, AVR::FullDuplex, true, 1>();
                    for(auto& o : output) {
                        o = (sbus_max + sbus_min) / 2;
                    }
                }

                static inline void set(const index_type& i, const value_type& v) {
                    output[i] = v;
                }
                template<uint8_t I>
                static inline void set(const value_type& v) {
                    output[I] = v;
                }
                
                static inline void switches(const std::byte s) {
                    static constexpr std::byte mask = ch17 | ch18;
                    mFlagsAndSwitches = (mFlagsAndSwitches & ~mask) | (s & mask);
                }
                
                static inline constexpr std::byte sbus_start = 0x0f_B;

                inline static void periodic() {
                    usart::periodic();
                }
                
                inline static void ratePeriodic() { // 14ms
                    (++ticks).on(timeoutTicks, []{
                        if constexpr(!std::is_same_v<dbg, void>) {
                            dbg::toggle();
                        }
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
                        usart::put(mFlagsAndSwitches); //Flags byte
                        usart::put(0x00_B); //Footer
                    });
                }
            private:
                static inline std::byte mFlagsAndSwitches{};
                static inline std::array<uint16_t, 16> output;                 
                static inline External::Tick<Timer> ticks{};
            };  

//            template<typename CN>
//            struct Generator {
                
//                using usart = AVR::Usart<CN, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<2>, AVR::SendQueueLength<128>>;
                
//                inline static constexpr uint16_t sbus_min = 172;
//                inline static constexpr uint16_t sbus_max = 1811;
                
//                inline static constexpr uint16_t sbus_mid = (sbus_max + sbus_min) / 2;
                
//                using value_type = etl::uint_ranged<uint16_t, sbus_min, sbus_max>;
                
//                inline static void init() {
//                    usart::template init<AVR::BaudRate<100000>, AVR::FullDuplex, true, 1>();
//                    for(auto& o : output) {
//                        o = (sbus_max + sbus_min) / 2;
//                    }
//                }

//                static inline constexpr std::byte sbus_start = 0x0f_B;

//                inline static void periodic() {
//                    usart::periodic();
//                }
                
//                inline static void ratePeriodic() { // 14ms
//                    usart::put(sbus_start);
                
//                    usart::put((std::byte) (output[0] & 0x07FF));
//                    usart::put((std::byte) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3));
//                    usart::put((std::byte) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6));
//                    usart::put((std::byte) ((output[2] & 0x07FF)>>2));
//                    usart::put((std::byte) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1));
//                    usart::put((std::byte) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4));
//                    usart::put((std::byte) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7));
//                    usart::put((std::byte) ((output[5] & 0x07FF)>>1));
//                    usart::put((std::byte) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2));
//                    usart::put((std::byte) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5));
//                    usart::put((std::byte) ((output[7] & 0x07FF)>>3));
//                    usart::put((std::byte) ((output[8] & 0x07FF)));
//                    usart::put((std::byte) ((output[8] & 0x07FF)>>8 | (output[9] & 0x07FF)<<3));
//                    usart::put((std::byte) ((output[9] & 0x07FF)>>5 | (output[10] & 0x07FF)<<6));  
//                    usart::put((std::byte) ((output[10] & 0x07FF)>>2));
//                    usart::put((std::byte) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1));
//                    usart::put((std::byte) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4));
//                    usart::put((std::byte) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7));
//                    usart::put((std::byte) ((output[13] & 0x07FF)>>1));
//                    usart::put((std::byte) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2));
//                    usart::put((std::byte) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5));
//                    usart::put((std::byte) ((output[15] & 0x07FF)>>3));
                
//                    usart::put(0x00_B); //Flags byte
//                    usart::put(0x00_B); //Footer
//                }
////            private:
//                static inline std::array<uint16_t, 16> output;                 
//            };  
        }
    }
}
