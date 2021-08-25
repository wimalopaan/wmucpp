#pragma once

#include <etl/format.h>

#include <external/solutions/rc/rf.h>

namespace External {
    namespace Multiprotocol {
        template<typename Dev, typename Timer, auto NumberOfSubprotocols>
        struct Generator {
            static_assert(NumberOfSubprotocols <= 8);
            static inline constexpr External::Tick<Timer> msendTicks{100_ms};
            
            inline static constexpr uint16_t sbus_min = 172;
            inline static constexpr uint16_t sbus_max = 1811;
            
            inline static constexpr uint16_t sbus_mid = (sbus_max + sbus_min) / 2;
            
            using value_type = etl::uint_ranged<uint16_t, sbus_min, sbus_max>;

            using Band = External::RC::Band;
            
            inline static void init() {
                output[0] = std::byte{'M'};
                output[1] = std::byte{'P'};
                output[2] = 0x01_B; // status
                output[3] = 24_B; // length
                output[4] = 0x00_B; // flags = data[0] invalid
                output[5] = 0x02_B; // major
                output[6] = 0x03_B; // minor
                output[7] = 0x04_B; // rev
                output[8] = 0x05_B; // patch
                output[9] = (0_B << 0) | (1_B << 2) | (2_B << 4) | (3_B << 6)   ; // AETR = data[5]
                output[10] = 88_B; // only this proto
                output[11] = 88_B;
                etl::StringBufferPart<12, 7, std::byte> sb(output);
                etl::copy(sb, "WM-FM-D"_pgm);
                output[19] = std::byte{8<<4 | (0x0f & NumberOfSubprotocols)}; // option RF freq. + subproto = data[15]
                
                band(Band::_40MHz);
            }

            inline static void band(const Band b) {
                switch(b) {
                case Band::_27MHz:
                {
                    etl::StringBufferPart<20, 8, std::byte> sb(output);
                    etl::copy(sb, "27M:"_pgm);
                }
                    break;
                case Band::_35AMHz:
                {
                    etl::StringBufferPart<20, 8, std::byte> sb(output);
                    etl::copy(sb, "35A:"_pgm);
                }
                    break;
                case Band::_35BMHz:
                {
                    etl::StringBufferPart<20, 8, std::byte> sb(output);
                    etl::copy(sb, "35B:"_pgm);
                }
                    break;
                case Band::_40MHz:
                {
                    etl::StringBufferPart<20, 8, std::byte> sb(output);
                    etl::copy(sb, "40M:"_pgm);
                }
                    break;
                case Band::_41MHz:
                {
                    etl::StringBufferPart<20, 8, std::byte> sb(output);
                    etl::copy(sb, "41M:"_pgm);
                }
                    break;
                case Band::_72MHz:
                {
                    etl::StringBufferPart<20, 8, std::byte> sb(output);
                    etl::copy(sb, "72M:"_pgm);
                }
                    break;
                }    
            }

            using ch_t = etl::uint_ranged<uint16_t, External::RC::minChannelNumber, External::RC::maxChannelNumber>;            

            inline static void channel(const ch_t& c) {
                etl::StringBufferPart<24, 4, etl::Char> sb(output);
                etl::itoa_r(c, sb);
            }
            
            inline static void valid(const bool b) {
                if (b) {
                    output[4] = 0x07_B; // flags: signale detected | serial mode | valid
                }
                else {
                    output[4] = 0x00_B;
                }
            }
            
            inline static void periodic() {
            }
            
            inline static void ratePeriodic() { 
                (++mSendTick).on(msendTicks, []{
                    for(const std::byte& b : output) {
                        Dev::put(b);
                    }
                });
            }
            private:
            
            static inline Band mBand{Band::_40MHz};
            
            static inline std::array<std::byte, 28> output{};                 
            static inline External::Tick<Timer> mSendTick{};
        };  
        
        template<auto N, typename Timer, typename Dbg = void>
        struct ProtocollAdapter {
            enum class State : uint8_t {Undefined, Data, GotStartLow, GotStartHigh, GotProto, GotSub};

            using data_t = std::array<uint16_t, 16>; 
            using value_type = etl::uint_ranged_NaN<uint16_t, 172, 1810>;
            using channel_t = etl::uint_ranged_NaN<uint8_t, 0, data_t::size() - 1>;
            
            static inline value_type value(const channel_t ch) {
                if (const uint8_t chi = ch.toInt(); ch) {
                    return mChannels[chi];
                }
                return value_type{};
            }

            static_assert(Timer::frequency >= 1000_Hz);
            
            static inline void ratePeriodic() {
                if (++tc > 5) {
                    mState = State::Undefined;
                }
            }
            
            static inline bool process(const std::byte b) {
                tc = 0;
                switch(mState) {
                case State::Undefined:
                    if (b == 0x55_B) {
                        mState = State::GotStartLow;
                    }
                    else if (b == 0x54_B) {
                        mState = State::GotStartHigh;
                    }
                    break;
                case State::GotStartLow:
                    if (const auto p = (b & 0x1F_B); p != 0x00_B) {
                        mProtoDrity = uint8_t(p);
                        mState = State::GotProto;
                    }
                    if (std::any(b & 0x40_B)) { // bind
                        mBind = true;
                    }
                    else {
                        mBind = false;
                    }
                    break;
                case State::GotStartHigh:
                    if (const auto p = (b & 0x1F_B); p != 0x00_B) {
                        mProtoDrity = uint8_t(p) + 32;
                        mState = State::GotProto;
                    }
                    if (std::any(b & 0x40_B)) { // bind
                        mBind = true;
                    }
                    else {
                        mBind = false;
                    }
                    break;
                case State::GotProto:
                    mSubProto.set<etl::RangeCheck<false>>(uint8_t((b >> 4) & 0x07_B)); // subproto [0,7]
                    mState = State::GotSub;
                    break;
                case State::GotSub: // option
                    if (const int8_t v = static_cast<int8_t>(b); (v >= 0) && (v <= 40)) { // the real range is [-128,128]
                        mState = State::Data;
                        mIndex.setToBottom();
                        rfChannel = v;
                    }
                    break;
                case State::Data:
                    mData[mIndex] = std::to_integer(b);
                    if (mIndex.isTop()) {
                        mState = State::Undefined;
                        decode();
                        mProtoHigh = mData[mData.size() - 1] & 0xc0;
                        mProto = mProtoDrity + mProtoHigh;
                        ++mPackages;
                    }
                    else {
                        ++mIndex;
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
            inline static uint8_t proto() {
                return mProto;
            }
            
            using subProto_t = etl::uint_ranged<uint8_t, 0, 7>;
            
            inline static subProto_t subProto() {
                return mSubProto;
            }
            inline static uint8_t channel() {
                return rfChannel;
            }
            inline static bool bind() {
                return mBind;
            }
            
        private:
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

            static inline uint8_t tc{0};
            
            static inline bool    mBind{}; 
            static inline uint8_t mProto{}; 
            static inline uint8_t mProtoDrity{}; 
            static inline uint8_t mProtoHigh{}; 
            static inline subProto_t mSubProto{}; 
            
            using MesgType = std::array<uint8_t, 23>;
            inline static data_t mChannels;
            inline static State mState{State::Undefined};
            inline static MesgType mData; 
            inline static etl::index_type_t<MesgType> mIndex;
            inline static uint16_t mPackages{};
            
            inline static uint8_t rfChannel{0};
        };

        namespace Deprecated {
            template<auto N, typename Timer, typename Dbg = void>
            struct ProtocollAdapter {
                enum class State : uint8_t {Undefined, Data, GotStart, GotProto, GotSub};
    
                using data_t = std::array<uint16_t, 16>; 
                using value_type = etl::uint_ranged_NaN<uint16_t, 172, 1810>;
                using mapped_type = etl::uint_ranged_NaN<uint16_t, 480, 1504>;
                using channel_t = etl::uint_ranged_NaN<uint8_t, 0, data_t::size() - 1>;
                
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
                    ++cc;
                    tc = 0;
                    switch(mState) {
                    case State::Undefined:
                        if (b == 0x54_B) {
                            mState = State::GotStart;
                        }
                        break;
                    case State::GotStart:
                        if ((b & 0x0F_B) == 0x05_B) { // Corona
                            mState = State::GotProto;
                        }
                        break;
                    case State::GotProto:
                        if (((b >> 4) & 0x07_B) == 0x00_B) { // subproto 0
                            mState = State::GotSub;
                        }
                        break;
                    case State::GotSub:
                        if (const int8_t v = static_cast<int8_t>(b); (v >= 0) && (v <= 40)) { 
                            mState = State::Data;
                            mIndex.setToBottom();
                            rfChannel = v;
                        }
                        break;
                    case State::Data:
                        mData[mIndex] = std::to_integer(b);
                        if (mIndex.isTop()) {
                            mState = State::Undefined;
                            decode();
                            ++mPackages;
                        }
                        else {
                            ++mIndex;
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
    //        private:
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
                inline static uint16_t mPackages{};
                inline static uint16_t cc{};
                
                inline static uint8_t rfChannel{0};
            };
        }        
    }
}
