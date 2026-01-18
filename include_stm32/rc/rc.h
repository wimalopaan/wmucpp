/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include <cstddef>
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <numbers>
#include <chrono>
#include <functional>

#include "byte.h"
#include "etl/algorithm.h"
#include "etl/ranged.h"
#include "units.h"
#include "tick.h"
#include "meta.h"

namespace RC {
    namespace Protokoll {
        namespace Null {
            template<typename Buf = void>
            class Adapter final {
            public:
                using buffer_t = Buf;
                using value_type = void;
                Adapter() = delete;
                static inline constexpr bool process(const std::byte) {
                    return false;
                }
                static inline void ratePeriodic() {}
            };
        }
        namespace IBus {
            
            namespace Type {
                // https://github.com/betaflight/betaflight/tree/master/src/main/telemetry
                
                // FlySky i6A: TEMPERATURE, RPM_FLYSKY, EXTERNAL_VOLTAGE
                
                enum class type : uint8_t {
                    NONE             = 0x00,
                    TEMPERATURE      = 0x01, // 0.1°C per step, -40°C offset (400 = 0°C)
                    RPM_FLYSKY       = 0x02,
                    EXTERNAL_VOLTAGE = 0x03, // 0.01V per step
                    CELL             = 0x04, // Avg Cell voltage
                    BAT_CURR         = 0x05, // battery current A * 100
                    FUEL             = 0x06, // remaining battery percentage / mah drawn otherwise or fuel level no unit!
                    RPM              = 0x07, // throttle value / battery capacity
                    CMP_HEAD         = 0x08, //Heading  0..360 deg, 0=north 2bytes
                    CLIMB_RATE       = 0x09, //2 bytes m/s *100
                    COG              = 0x0a, //2 bytes  Course over ground(NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. unknown max uint
                    GPS_STATUS       = 0x0b, //2 bytes
                    ACC_X            = 0x0c, //2 bytes m/s *100 signed
                    ACC_Y            = 0x0d, //2 bytes m/s *100 signed
                    ACC_Z            = 0x0e, //2 bytes m/s *100 signed
                    ROLL             = 0x0f, //2 bytes deg *100 signed
                    PITCH            = 0x10, //2 bytes deg *100 signed
                    YAW              = 0x11, //2 bytes deg *100 signed
                    VERTICAL_SPEED   = 0x12, //2 bytes m/s *100
                    GROUND_SPEED     = 0x13, //2 bytes m/s *100 different unit than build-in sensor
                    GPS_DIST         = 0x14, //2 bytes dist from home m unsigned
                    ARMED            = 0x15, //2 bytes
                    FLIGHT_MODE      = 0x16, //2 bytes
                    PRES             = 0x41, // Pressure
        
                    ANGLE            = 0x70,
                    FW_VERSION          = 0x71,
                    STATE            = 0x72,
                    
                    ODO1             = 0x7c, // Odometer1
                    ODO2             = 0x7d, // Odometer2
                    SPEED              = 0x7e, // Speed 2bytes km/h
                    
                    GPS_LAT          = 0x80, //4bytes signed WGS84 in degrees * 1E7
                    GPS_LON          = 0x81, //4bytes signed WGS84 in degrees * 1E7
                    GPS_ALT          = 0x82, //4bytes signed!!! GPS alt m*100
                    ALT              = 0x83, //4bytes signed!!! Alt m*100
                    ALT_MAX          = 0x84, //4bytes signed MaxAlt m*100
                    
                    ID_S85           = 0x85,     
                    ID_S86           = 0x86,     
                    ID_S87           = 0x87,     
                    ID_S88           = 0x85,     
                    ID_S89           = 0x89,     
                    ID_S8a           = 0x8a,     
                    
                    ALT_FLYSKY       = 0xf9, // Altitude 2 bytes signed in m
                    
                    RX_SNR           = 0xfa,
                    RX_NOISE         = 0xfb,
                    RX_RSSI          = 0xfc,
                    RX_ERR_RATE      = 0xfe,
                    
                    UNKNOWN          = 0xff,
                    END              = 0xff
                };
            }

            struct CheckSum final {
                inline void reset() {
                    mSum = std::numeric_limits<uint16_t>::max();
                }
                inline std::byte operator+=(const std::byte b) {
                    mSum -= static_cast<uint8_t>(b);
                    return b;
                }
                inline std::byte highByte() const {
                    return std::byte(mSum >> 8);
                }
                inline std::byte lowByte() const {
                    return std::byte(mSum);
                }
                inline void highByte(const std::byte hb) {
                    mH = hb;
                }
                inline void lowByte(const std::byte lb){
                    mL = lb;
                }
                inline explicit operator bool() const {
                    return (etl::nth_byte<0>(mSum) == mL) && (etl::nth_byte<1>(mSum) == mH);
                }
            private:
                std::byte mH{};
                std::byte mL{};
                uint16_t mSum = std::numeric_limits<uint16_t>::max();
            };

            template<auto N, typename Dbg = void>
            struct Adapter {
                enum class State : uint8_t {Undefined, GotStart20, Data, CheckL, CheckH};
                
                using value_type = etl::ranged_NaN<988, 2011>;
                using normalized_type = etl::ranged_NaN<-1000, +1000>;
                using channel_type = etl::ranged<0, 17>;
                
                static inline normalized_type normalized(const uint8_t ch) {
                    if (ch < 18) {
                        const float d = value(channel_type{ch}) - 1500;
                        const float nd = d * 1000.0f / 511.0f;
                        return normalized_type(nd);
                    }
                    return normalized_type{0};
                }

                static inline value_type value(const channel_type ch) {
                    if (ch < 14) {
                        const std::byte h = (*inactive)[ch * 2 + 1] & std::byte{0x0f};
                        const std::byte l = (*inactive)[ch * 2];
                        const uint16_t  v = (uint16_t(h) << 8) + uint8_t(l);
                        if ((v >= value_type::Lower) && (v <= value_type::Upper)) {
                            return value_type{v};
                        }
                    }
                    else if (ch < 18) {
                        const std::byte h1 = (*inactive)[6 * (ch - 14) + 1] & std::byte{0xf0};
                        const std::byte h2 = (*inactive)[6 * (ch - 14) + 3] & std::byte{0xf0};
                        const std::byte h3 = (*inactive)[6 * (ch - 14) + 5] & std::byte{0xf0};
                        const uint16_t v = (uint8_t(h1) >> 4) + uint8_t(h2) + (uint16_t(h3) << 4);
                        if ((v >= value_type::Lower) && (v <= value_type::Upper)) {
                            return value_type{v};
                        }
                    }
                    else {
                        return {};
                    }
                    return {};
                }
                static inline bool process(const std::byte b) {
                    switch(mState) {
                    case State::Undefined:
                        csum.reset();
                        if (b == std::byte{0x20}) {
                            csum += b;
                            mState = State::GotStart20;
                        }
                        break;
                    case State::GotStart20:
                        if (b == std::byte{0x40}) {
                            csum += b;
                            mState = State::Data;
                            mIndex.toBottom();
                        }
                        else {
                            mState = State::Undefined;
                        }
                        break;
                    case State::Data:
                        (*active)[mIndex] = b;
                        csum += b;
                        if (mIndex.isTop()) {
                            mState = State::CheckL;
                        }
                        else {
                            ++mIndex;
                        }
                        break;
                    case State::CheckL:
                        csum.lowByte(b);
                        mState = State::CheckH;
                        break;
                    case State::CheckH:
                        csum.highByte(b);
                        mState = State::Undefined;
                        if (csum) {
                            ++mPackagesCounter;
                            if constexpr (!std::is_same_v<Dbg, void>) {
                                Dbg::toggle();
                            }
                            using std::swap;
                            swap(active, inactive);
                        }
                        break;
                    }
                    return true;
                }
                
                inline static void ratePeriodic() {}
                
                inline static void resetStats() {
                    mPackagesCounter = 0;
                }
                inline static uint16_t packages() {
                    return mPackagesCounter;
                }
            private:
                using MesgType = std::array<std::byte, 28>;
                using index_type = etl::index_type_t<MesgType>;
                
                inline static CheckSum csum;
                inline static State mState{State::Undefined};
                inline static MesgType mData0; // 0x20, 0x40 , 28 Bytes, checkH, checkL
                inline static MesgType mData1; // 0x20, 0x40 , 28 Bytes, checkH, checkL
                inline static MesgType* active = &mData0;
                inline static MesgType* inactive = &mData1;
                inline static index_type mIndex;
                inline static uint16_t mPackagesCounter{};
            };
            
            
            using namespace etl::literals;
            using namespace std::literals::chrono_literals;
            using namespace Units::literals;
            
            
            template<
                     template<typename PA> typename Uart, 
                     typename Clock,
                     typename ProviderList,
                     typename DaisyChainEnable = void,
                     bool useLoss = false,
                     bool useStats = false,
                     typename Debug = void> 
            struct  Sensor;
            
            template<
                     template<typename PA> typename Uart, 
                     typename Clock,
                     typename... Providers,
                     typename DaisyChainEnable,
                     bool useLoss,
                     bool useStats,
                     typename Debug>
            struct Sensor<Uart, Clock, Meta::List<Providers...>, DaisyChainEnable, useLoss, useStats, Debug> final {
                struct NoDebug {
                    static inline void init() {}
                    template<auto T>
                    static inline constexpr void set() {}
                    static inline constexpr void set(std::byte) {}
                };
                
                struct LossProvider {
                    inline static constexpr auto ibus_type = Type::type::FLIGHT_MODE;
                    inline static constexpr void init() {}
                    inline static constexpr uint16_t value() {
                        return 10000 + (lossCounter * 100) + noQueriesCounter;
                    }
                    inline static void incLoss() {
                        if (++lossCounter == 100) {
                            lossCounter = 0;
                        }
                    }
                    inline static void incNoQuery() {
                        if (++noQueriesCounter == 100) {
                            noQueriesCounter = 0;
                        }
                    }
                    static inline uint8_t lossCounter{};
                    static inline uint8_t noQueriesCounter{};
                };
                struct StatisticProvider {
                    inline static constexpr auto ibus_type = Type::type::FLIGHT_MODE;
                    inline static constexpr void init() {}
                    inline static constexpr uint16_t value() {
                        return 10000 + (mPackets * 100) + mBytes;
                    }
                    inline static constexpr void incBytes() {
                        if (++mBytes == 100) {
                            mBytes = 0;
                        }
                    }
                    inline static constexpr void incPackets() {
                        if (++mPackets == 100) {
                            mPackets = 0;
                        }
                    }
                    static inline uint8_t mBytes{};
                    static inline uint8_t mPackets{};
                };
                
                using debug = std::conditional_t<std::is_same_v<Debug, void>, NoDebug, Debug>;
                
                using provider_list_extern = Meta::List<Providers...>;
        
                using provider_list1 = std::conditional_t<useLoss, 
                                                         Meta::push_back<provider_list_extern, LossProvider>,
                                                         provider_list_extern>;
                using provider_list = std::conditional_t<useStats, 
                                                         Meta::push_back<provider_list1, StatisticProvider>,
                                                         provider_list1>;
                
                
                inline static constexpr auto numberOfProviders = Meta::size_v<provider_list>;
                
                inline static constexpr std::byte Cdiscover = 0x80_B;
                inline static constexpr std::byte CgetType  = 0x90_B;
                inline static constexpr std::byte CgetValue = 0xa0_B;
                inline static constexpr std::byte Creset    = 0xf0_B;
                
                using sensor_number_t = etl::ranged<1, 15>;
        
        //        using prov_type = etl::uint_ranged<uint8_t, 0, numberOfProviders - 1>;        
                inline static void maxProvider(const uint8_t n) {
                        mEnabledProviders = std::min(n, numberOfProviders);
                }
                
                struct ProtocollAdapter final {
                    inline static constexpr uint8_t   Length    = 4;
                    
                    enum class ibus_state_t {Undefined = 0, Reset, 
                                             Length, Discover, Type, Value, Skip, 
                                             CheckSum, CheckSumSkip,
                                             Reply};
                    
                    ProtocollAdapter() = delete;
                    
                    inline static void start() {
                        mState = ibus_state_t::Undefined;
                        uart::template rxEnable<true>();
                    }
                    inline static void reset() {
                        mFirstSensorNumber= sensor_number_t{};
                        mLastSensorNumber = sensor_number_t{};
                        mReceivedNumber   = sensor_number_t{};
                        if constexpr(!std::is_same_v<DaisyChainEnable, void>) {
                            DaisyChainEnable::off();
                        }
                        start();
                    }
        
                    inline static constexpr auto requests() {
                        return StatisticProvider::mPackets;
                    }
                    
                    static constexpr External::Tick<Clock> timeoutTicks{300ms};
                    
                    inline static bool process(const std::byte c) {
        //                debug::template set<1>();
                        StatisticProvider::incBytes();
                        switch (mState) {
                        case ibus_state_t::Undefined:
                            csum.reset();
                            responder::start();
                            if (length(c) == Length) {
                                mState = ibus_state_t::Length;
                                csum += c;
                            }
                            break;
                        case ibus_state_t::Reset:
                            reset();
                            mState = ibus_state_t::CheckSumSkip;
                            break;
                        case ibus_state_t::Length:
                            StatisticProvider::incPackets(); 
                            csum += c;  
                            mReceivedNumber = sensor_number_t{};
                            if (command(c) == Cdiscover) {
                                debug::template set<1>();
                                if (const sensor_number_t n{address(c)}; n) {
                                    if (!mFirstSensorNumber) {
                                        mFirstSensorNumber = n;
                                        if (mEnabledProviders == 1) {
                                            mLastSensorNumber = n;
                                        }
                                        mReceivedNumber = n;
                                        mState = ibus_state_t::Discover;
                                        responder::start(responder::reply_state_t::DiscoverWait);
                                    }
                                    else if (!mLastSensorNumber) {
                                        const uint8_t index = n - mFirstSensorNumber;
                                        if (index == (mEnabledProviders - 1)) {
                                            mLastSensorNumber = n;
                                        }
                                        mReceivedNumber = n;
                                        mState = ibus_state_t::Discover;
                                        responder::start(responder::reply_state_t::DiscoverWait);
                                    }
                                    else if (mFirstSensorNumber && mLastSensorNumber && inRange(n)) { // maybe a loss of connection
                                        mReceivedNumber = n;
                                        mState = ibus_state_t::Discover;
                                        responder::start(responder::reply_state_t::DiscoverWait);
                                        LossProvider::incLoss();
                                    }
                                    else {
                                        mState = ibus_state_t::Skip;
                                    }
                                }
                                else {
                                    mState = ibus_state_t::Undefined;
                                }
                            }
                            else if (command(c) == CgetType) {
                                if (const sensor_number_t n{address(c)}; n) {
                                    if (inRange(n)) {
                                        mReceivedNumber = n;
                                        mState = ibus_state_t::Type;                        
                                        responder::start(responder::reply_state_t::TypeWait);
                                    }
                                    else {
                                        mState = ibus_state_t::Skip;
                                    }
                                }
                                else {
                                    mState = ibus_state_t::Undefined;
                                }
                            }
                            else if (command(c) == CgetValue) {
                                if (const sensor_number_t n{address(c)}; n) {
                                    if (inRange(n)) {
                                        ++mQueries;
                                        mReceivedNumber = n;
                                        mState = ibus_state_t::Value;                        
                                        responder::start(responder::reply_state_t::ValueWait);
                                    }
                                    else {
                                        mState = ibus_state_t::Skip;
                                    }
                                }
                                else {
                                    mState = ibus_state_t::Undefined;
                                }
                            }
        //                    else if (command(c) == Creset) {
                            else if (c == Creset) {
                                mState = ibus_state_t::Reset;                        
                            }
                            else {
                                mState = ibus_state_t::Undefined;
                            }
                            break;
                        case ibus_state_t::Skip:
                            mState = ibus_state_t::CheckSumSkip;                        
                            break;
                        case ibus_state_t::Discover:
                            csum.lowByte(c);
                            mState = ibus_state_t::CheckSum;                        
                            break;
                        case ibus_state_t::Type:
                            csum.lowByte(c);
                            mState = ibus_state_t::CheckSum;                        
                            break;
                        case ibus_state_t::Value:
                            csum.lowByte(c);
                            mState = ibus_state_t::CheckSum;                        
                            break;
                        case ibus_state_t::CheckSum:
                            csum.highByte(c);
                            if (csum) { // mReceivedNumber valid
                                mState = ibus_state_t::Reply;                        
                                uart::template rxEnable<false>();
                            }
                            else {
                                mState = ibus_state_t::Undefined;
                            }
                            break;
                        case ibus_state_t::CheckSumSkip:
                            mState = ibus_state_t::Undefined;                        
                            break;
                        case ibus_state_t::Reply:
                            break;
                        default:
                            break;
                        }
        //                debug::template set<0>();
                        debug::set(std::byte(mState));
                        return true;
                    }
                    inline static constexpr bool permitReply() {
                        return mState == ibus_state_t::Reply;
                    }
                    inline static void ratePeriodic() {
                        static uint16_t lastQueries{};
                        stateTicks.on(timeoutTicks, [&]{
                            if (mQueries == lastQueries) {
                                reset();
                                LossProvider::incNoQuery();
                            }
                            lastQueries = mQueries;
                        });
        //                switch(mState) {
        //                case ibus_state_t::Undefined:   
        //                case ibus_state_t::Reset:   
        //                case ibus_state_t::Length:   
        //                case ibus_state_t::Discover:   
        //                case ibus_state_t::Type:   
        //                case ibus_state_t::Value:   
        //                case ibus_state_t::Skip:   
        //                case ibus_state_t::CheckSum:   
        //                case ibus_state_t::CheckSumSkip:   
        //                case ibus_state_t::Reply:
        //                    break;
        //                }
                    }
                private:
                    static inline constexpr std::byte command(const std::byte b) {
                        if ((b & 0x0f_B) == 0x00_B) {
                            return 0x00_B;
                        }
                        return (b & 0xf0_B) ;
                    }
                    static inline constexpr uint8_t length(const std::byte b) {
                        if ((b & 0xf0_B) != 0x00_B) {
                            return {};
                        }
                        return uint8_t(b & 0x0f_B);
                    }
                    static inline constexpr sensor_number_t address(const std::byte b) {
                        auto v = uint8_t(b & 0x0f_B);
                        if (v != 0) {
                            return sensor_number_t{v};
                        }
                        else {
                            return {};
                        }
                    }
                    static inline uint16_t mQueries{0};
                    static inline External::Tick<Clock> stateTicks{};
                    static inline CheckSum csum;
                    static inline ibus_state_t  mState = ibus_state_t::Undefined;
                };
                
                using pa = ProtocollAdapter;
                using uart = Uart<pa>;
        
                inline static constexpr void init() {
                    (Providers::init(), ...);
                    uart::init();
                    uart::baud(115200);
                    uart::halfDuplex();
//                    uart::template init<Baud, AVR::HalfDuplex>();
                    if constexpr(!std::is_same_v<DaisyChainEnable, void>) {
                        DaisyChainEnable::init();
                    }
                    clear();
                }
                inline static constexpr void rxtxswap(const bool b) {
                    uart::rxtxswap(b);
                }

                inline static constexpr void clear() {
                    if constexpr(!std::is_same_v<DaisyChainEnable, void>) {
                        DaisyChainEnable::off();
                    }
                    pa::reset();
                    responder::start();
                    debug::init();
        //            debug::template set<0x00>();
                }
                
                struct Responder final {
                    inline static constexpr auto delayBeforeReply = 700us;
                    static inline constexpr auto intervall = Clock::intervall;
        //                                std::integral_constant<uint16_t, intervall.value>::_;
                    static inline constexpr auto ticks_to_wait = delayBeforeReply / intervall;
//                                        std::integral_constant<uint8_t, ticks_to_wait>::_;
                    static_assert((ticks_to_wait > 0) || (intervall <= 1000us));        
                    using wait_t = etl::ranged<0, ticks_to_wait+1>;
                    
                    enum class reply_state_t {Undefined = 0, 
                                              DiscoverWait, TypeWait, ValueWait, 
                                              Discover, Type, Value,
                                              DaisyWait, DaisySet,
                                              Wait, WaitOver};
                    
                    Responder() = delete;
                    inline static constexpr void start(const reply_state_t s = reply_state_t::Undefined) {
                        mReply = s;
                        mTicks.toBottom(); 
//                        mTicks.template set<etl::RangeCheck<false>>(0); 
                    }
                    inline static constexpr void ratePeriodic() {
                        if (pa::permitReply()) {
                            ++mTicks;
                            if (mTicks.isTop()) {
                                switch(mReply) {
                                case reply_state_t::DiscoverWait:
                                    mReply = reply_state_t::Discover;
                                    break;
                                case reply_state_t::TypeWait:
                                    mReply = reply_state_t::Type;
                                    break;
                                case reply_state_t::ValueWait:
                                    mReply = reply_state_t::Value;
                                    break;
                                case reply_state_t::Undefined:
                                case reply_state_t::Discover:
                                case reply_state_t::Type:
                                case reply_state_t::Value:
                                case reply_state_t::Wait:
                                case reply_state_t::WaitOver:
                                case reply_state_t::DaisyWait:
                                case reply_state_t::DaisySet:
                                    break;
                                default:
                                    break;
                                }
                                debug::set(std::byte(uint8_t(mReply) + 16));
                            }
                        }
                    }
                    inline static constexpr void periodic() {
                        uart::periodic();
                        switch(mReply) {
                        case reply_state_t::Discover:
                            if (inRange(mReceivedNumber)) {
                                CheckSum cs;
                                uart::put(cs += 0x04_B);
                                uart::put(cs += Cdiscover | std::byte((uint8_t)mReceivedNumber));
                                uart::put(cs.lowByte());
                                uart::put(cs.highByte());
                                mReply = reply_state_t::Wait;
                                debug::set(std::byte(uint8_t(mReply) + 16));
                            }
                            break;
                        case reply_state_t::Type:
                            if (inRange(mReceivedNumber)) {
                                uint8_t index = mReceivedNumber - mFirstSensorNumber;
                                auto type = Type::type::NONE;
                                Meta::visitAt<provider_list>(index, [&]<typename P>(Meta::Wrapper<P>){
                                                                 type = P::ibus_type;
                                                             });
                                CheckSum cs;
                                uart::put(cs += 0x06_B);
                                uart::put(cs += CgetType | std::byte((uint8_t)mReceivedNumber));
                                uart::put(cs += std::byte(type));
                                uart::put(cs += 0x02_B); // fix
                                uart::put(cs.lowByte());
                                uart::put(cs.highByte());
                                mReply = reply_state_t::DaisyWait;
                                debug::set(std::byte(uint8_t(mReply) + 16));
                            }
                            break;
                        case reply_state_t::Value:
                            if (inRange(mReceivedNumber)) {
                                uint8_t index = mReceivedNumber - mFirstSensorNumber;
                                uint16_t value{};
                                Meta::visitAt<provider_list>(index, [&]<typename P>(Meta::Wrapper<P>){
                                                                 value = P::value();
                                                             });
                                CheckSum cs;
                                uart::put(cs += 0x06_B);
                                uart::put(cs += CgetValue | std::byte{(uint8_t)mReceivedNumber});
                                uart::put(cs += etl::nth_byte<0>(value));
                                uart::put(cs += etl::nth_byte<1>(value)); 
                                uart::put(cs.lowByte());
                                uart::put(cs.highByte());
                                mReply = reply_state_t::Wait;
                                debug::set(std::byte(uint8_t(mReply) + 16));
                            }
                            break;
                        case reply_state_t::Wait:
                            if (uart::isIdle()) {
                                mReply = reply_state_t::WaitOver;
                                debug::set(std::byte(uint8_t(mReply) + 16));
                            }
                            break;
                        case reply_state_t::DaisyWait:
                            if (uart::isIdle()) {
                                mReply = reply_state_t::DaisySet;
                                debug::set(std::byte(uint8_t(mReply) + 16));
                            }
                            break;
                        case reply_state_t::WaitOver:
                            pa::start();
                            mReply = reply_state_t::Undefined;
                            debug::set(std::byte(uint8_t(mReply) + 16));
                            break;
                        case reply_state_t::DaisySet:
                            pa::start();
                            mReply = reply_state_t::Undefined;
                            if constexpr(!std::is_same_v<DaisyChainEnable, void>) {
                                if (mLastSensorNumber) {
                                    DaisyChainEnable::on();
                                }
                            }
                            debug::set(std::byte(uint8_t(mReply) + 16));
                            break;
                        case reply_state_t::Undefined:
                        case reply_state_t::DiscoverWait:
                        case reply_state_t::TypeWait:
                        case reply_state_t::ValueWait:
                            break;
                        default:
                            break;
                        }
                    }
                private:
                    static inline reply_state_t mReply = reply_state_t::Undefined;
                    inline static wait_t mTicks;
                };
                
                using responder = Responder;
                
                inline static constexpr void ratePeriodic() {
                    pa::ratePeriodic();
                    responder::ratePeriodic();
                }
                inline static constexpr void periodic() {
                    responder::periodic();
                }
                
                private:
                static inline constexpr bool inRange(const sensor_number_t n) {
                    if (!n) return false;
                    const auto in = n;
                    return (in >= mFirstSensorNumber) && (in < (mFirstSensorNumber + mEnabledProviders));
                }
                
                static inline sensor_number_t mReceivedNumber;
                static inline sensor_number_t mFirstSensorNumber;
                static inline sensor_number_t mLastSensorNumber;
        
                static inline uint8_t mEnabledProviders{numberOfProviders};
            };
            
            
            
            
        }
        
        namespace SBus {
            using namespace etl::literals;
            static inline constexpr std::byte start_byte = 0x0f_B;
            static inline constexpr std::byte end_byte = 0x00_B;
            
            static inline constexpr std::byte ch17 = 0x01_B;
            static inline constexpr std::byte ch18 = 0x02_B;
            static inline constexpr std::byte frameLost = 0x04_B;
            static inline constexpr std::byte failSafe = 0x08_B;
    
            namespace Servo {
                using namespace std::literals::chrono_literals;
                using namespace Units::literals;
                
                template<auto N, typename Timer, typename Dbg = void>
                struct Adapter {
                    enum class State : uint8_t {Undefined, Data, GotEnd, WaitEnd};
    
                    using data_t = std::array<uint16_t, 16>; 
                    using value_type = etl::ranged_NaN<172, 1811>;
//                    using mapped_type = etl::ranged_NaN<480, 1504>;
                    using channel_type = etl::ranged<0, std::tuple_size_v<data_t> - 1>;
                    
                    static inline std::byte switches() {
                        return mFlagsAndSwitches & (ch17 | ch18);
                    }
                    
                    static inline std::byte flags() {
                        return mFlagsAndSwitches & (failSafe | frameLost);
                    }
                    
                    static inline value_type value(const channel_type ch) {
                        return value_type{mChannels[ch]};
                        if (mValid) {
                            return value_type{mChannels[ch]};
                        }
//                        if (const uint8_t chi = ch.toInt(); ch) {
//                            if (mValid) {
//                                return value_type{mChannels[chi]};
//                            }
//                        }
                        return value_type{};
                    }
//                    static inline mapped_type valueMapped(const channel_t ch) {
//                        if (const uint8_t chi = ch.toInt(); ch) {
//                            if (mValid) {
//                                return mChannels[chi];
//                            }
//                        }
//                        return mapped_type{};
//                    }
    
                    static_assert(Timer::frequency >= 1000_Hz);
                    
                    static inline constexpr External::Tick<Timer> byteTimeout{2ms};
                    static inline constexpr External::Tick<Timer> packageTimeout{500ms};
    
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
                        ++mBytes;
                        mByteTimer.reset();
                        switch(mState) {
                        case State::Undefined:
                            if (b == end_byte) {
                                mState = State::GotEnd;
                            }
                            else if (b == start_byte) {
                                ++mStart;
                                mState = State::Data;
                                mIndex.toBottom();
                            }
                            break;
                        case State::GotEnd:
                            if (b == start_byte) {
                                mState = State::Data;
                                mIndex.toBottom();
                            }
                            else if (b == end_byte) {
                                mState = State::GotEnd;
                            }
                            else {
                                mState = State::Undefined;
                            }
                            break;
                        case State::Data:
//                            mData[mIndex] = std::to_integer(b);
                            mData[mIndex] = MesgType::value_type(b);
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
                    inline static uint16_t bytes() {
                        return mBytes;
                    }
                    inline static uint16_t starts() {
                        return mStart;
                    }
                    inline static void resetStats() {
                        mPackages = 0;
                        mBytes = 0;
                        mStart = 0;
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
                    inline static uint16_t mBytes{};
                    inline static uint16_t mStart{};
                    inline static bool mValid{false};
                    inline static std::byte mFlagsAndSwitches{0};
                };
            }

            namespace Output {
                // using namespace std::literals::chrono;
                using namespace std::literals::chrono_literals;
                
                template<typename Usart, typename Timer, typename dbg = void>
                struct Generator {
                    using usart = Usart;
                    static constexpr External::Tick<Timer> timeoutTicks{14ms};
                    // static_assert(timeoutTicks.value > 1);
    //                std::integral_constant<uint8_t, timeoutTicks.value>::_;
                    
                    inline static constexpr uint16_t sbus_min = 172;
                    inline static constexpr uint16_t sbus_max = 1811;
                    inline static constexpr uint16_t sbus_span = (sbus_max - sbus_min) / 2;
                    
                    inline static constexpr uint16_t sbus_mid = (sbus_max + sbus_min) / 2;
                    
                    using value_type = etl::ranged<sbus_min, sbus_max>;
                    using index_type = etl::ranged<0, 15>;
                    
                    inline static void init() {
                        // usart::template init<AVR::BaudRate<100000>, AVR::FullDuplex, true, 1>();
                        for(auto& o : output) {
                            o = (sbus_max + sbus_min) / 2;
                        }
                    }
                    static inline void set(const uint8_t i, const int8_t v) {
                        output[i] = sbus_mid + v * ((float)sbus_span) / 128.0f;
                    }

                    static inline void set(const index_type& i, const value_type& v) {
                        output[i] = v;
                    }
                    template<uint8_t I>
                    static inline void set(const value_type& v) {
                        output[I] = v;
                    }
                    
                    template<typename C>
                    static inline void set(const C& v) {
                        etl::copy(v, output);
                    }
                    
                    static inline void switches(const std::byte s) {
                        static constexpr std::byte mask = ch17 | ch18;
                        mFlagsAndSwitches = (mFlagsAndSwitches & ~mask) | (s & mask);
                    }
                    
                    static inline constexpr std::byte sbus_start = 0x0f_B;
    
                    inline static void periodic() {
                        usart::periodic();
                    }
                    
                    inline static void start() {
                        mActive = true;
                    }
                    inline static void stop() {
                        mActive = false;
                    }

                    inline static void ratePeriodic() { // 14ms
                        if (!mActive) return;

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
                            usart::put(0x04_B); //Footer
                        });
                    }
                private:
                    static inline bool mActive{true};
                    static inline std::byte mFlagsAndSwitches{};
                    static inline std::array<uint16_t, 16> output;                 
                    static inline External::Tick<Timer> ticks{};
                }; 
            }
        }
        namespace Hott {            
            namespace SumDV3 {
                static constexpr uint32_t baudrate{115200};

                static constexpr uint8_t start_code = 0xa8;
                static constexpr uint8_t version_code1 = 0x01;
                static constexpr uint8_t version_code3 = 0x03;

                static constexpr uint16_t ExtendedLow = 0x1c20; // 7200
                static constexpr uint16_t MinValue = 0x2260; // 8800
                static constexpr uint16_t CenterValue = 0x2ee0; // 12000
                static constexpr uint16_t MaxValue = 0x3b60; // 15200
                static constexpr uint16_t ExtendedHigh = 0x41a0; // 16800

                static constexpr uint8_t MaxChannels = 32;

                using MesgType = std::array<std::pair<uint8_t, uint8_t>, MaxChannels>;
                using SwitchesType = uint64_t;
                using Command_t = std::pair<uint8_t, uint8_t>;


                struct Crc16 {
                    inline void reset() {
                        sum = 0;
                    }
                    inline void operator+=(const uint8_t v) {
                        sum = sum ^ (((uint16_t)v) << 8);
                        for(uint8_t i = 0; i < 8; ++i) {
                            if (sum & 0x8000) {
                                sum = (sum << 1) ^ crc_polynome;
                            }
                            else {
                                sum = (sum << 1);
                            }
                        }
                    }
                    inline operator uint16_t() const {
                        return sum;
                    }
                private:
                    static constexpr uint16_t crc_polynome = 0x1021;
                    uint16_t sum{};
                };

                template<uint8_t Instance>
                struct Servo {
                    using MesgType = SumDV3::MesgType;
                    using SwitchesType = SumDV3::SwitchesType;
                    using Command_t = SumDV3::Command_t;

                    template<uint8_t B, uint8_t E> struct range_t {};
                    template<uint8_t N> using offset_t = std::integral_constant<uint8_t, N>;

                    enum class State : uint8_t {Undefined, GotStart, StartV1, StartV3, V1ChannelDataH, V1ChannelDataL, CrcH, CrcL,
                                               V3ChannelDataH, V3ChannelDataL, V3FuncCode, V3LastValidPackage, V3ModeCmd, V3SubCmd};

                    enum class Frame : uint8_t {Ch1to12 = 0x00, First = Ch1to12,
                                                Ch1to8and13to16 = 0x01, Ch1to16 = 0x02, Ch1to8and17to24 = 0x03,
                                                Ch1to8and25to32 = 0x04, Ch1to12and64Switches = 0x05,
                                                Last = Ch1to12and64Switches,
                                                Undefined = 0xff};

                    static inline void process(const uint8_t b, const std::function<void()> f) {
                        ++mBytesCounter;
                        switch(mState) { // enum-switch -> no default (intentional)
                        case State::Undefined:
                            if (b == SumDV3::start_code) {
                                csum.reset();
                                csum += b;
                                mState = State::GotStart;
                            }
                            break;
                        case State::GotStart:
                            csum += b;
                            if ((b & 0x0f) == SumDV3::version_code1) {
                                mState = State::StartV1;
                            }
                            else if ((b & 0x0f) == SumDV3::version_code3) {
                                mState = State::StartV3;
                            }
                            else {
                                mState = State::Undefined;
                            }
                            break;
                        case State::StartV1:
                            if ((b >= 2) && (b <= 32)) {
                                csum += b;
                                nChannels = b;
                                mIndex = 0;
                                mState = State::V1ChannelDataH;
                            }
                            else {
                                mState = State::Undefined;
                            }
                            break;
                        case State::V1ChannelDataH:
                            csum += b;
                            sumdFrame[mIndex].first = b;
                            mState = State::V1ChannelDataL;
                            break;
                        case State::V1ChannelDataL:
                            csum += b;
                            sumdFrame[mIndex].second = b;
                            ++mIndex;
                            if (mIndex < nChannels) {
                                mState = State::V1ChannelDataH;
                            }
                            else {
                                mState = State::CrcH;
                                frame = Frame::Ch1to16;
                            }
                            break;
                        case State::CrcH:
                            crcH = b;
                            mState = State::CrcL;
                            break;
                        case State::CrcL:
                            if (((((uint16_t)crcH) << 8) | b) == csum) {
                                ++mPackagesCounter;
                                f();
                            }
                            mState = State::Undefined;
                            break;
                        case State::StartV3:
                            if ((b >= 2) && (b <= 68)) {
                                csum += b;
                                nChannels = b - 2;
                                mIndex = 0;
                                mState = State::V3ChannelDataH;
                            }
                            else {
                                mState = State::Undefined;
                            }
                            break;
                        case State::V3ChannelDataH:
                            csum += b;
                            sumdFrame[mIndex].first = b;
                            mState = State::V3ChannelDataL;
                            break;
                        case State::V3ChannelDataL:
                            csum += b;
                            sumdFrame[mIndex].second = b;
                            ++mIndex;
                            if (mIndex < nChannels) {
                                mState = State::V3ChannelDataH;
                            }
                            else {
                                mState = State::V3FuncCode;
                            }
                            break;
                        case State::V3FuncCode:
                            csum += b;
                            if ((b >= uint8_t(Frame::First)) && (b <= uint8_t(Frame::Last))) {
                                frame = Frame(b);
                            }
                            else {
                                frame = Frame::Undefined;
                            }
                            mState = State::V3LastValidPackage;
                            break;
                        case State::V3LastValidPackage:
                            csum += b;
                            reserved = b;
                            mState = State::V3ModeCmd;
                            break;
                        case State::V3ModeCmd:
                            csum += b;
                            mode_cmd = b;
                            mState = State::V3SubCmd;
                            break;
                        case State::V3SubCmd:
                            csum += b;
                            sub_cmd = b;
                            if (!hasCommand()) {
                                mCommand = Command_t{mode_cmd, sub_cmd};
                            }
                            mState = State::CrcH;
                            break;
                        }
                    }

                    template<uint8_t N>
                    static inline void convert(int16_t (&pulses)[N]) {
                        static_assert(N >= 16, "array too small");
                        switch(frame) {
                        case Frame::Ch1to12:
                            extract(range_t<0, 11>{}, pulses);
                            break;
                        case Frame::Ch1to8and13to16:
                            extract(range_t<0, 7>{}, pulses);
                            extract(range_t<8, 11>{}, pulses, offset_t<12>{});
                            break;
                        case Frame::Ch1to8and17to24:
                            extract(range_t<0, 7>{}, pulses);
                            extract(range_t<8, 15>{}, pulses, offset_t<16>{}, std::integral_constant<bool, (N > 16) >{}); // no constepr-if in c++11
            //                if constexpr(N > 16) {
            //                    extract(range_t<8, 15>{}, pulses, offset_t<16>{});
            //                }
                            break;
                        case Frame::Ch1to8and25to32:
                            extract(range_t<0, 7>{}, pulses);
                            extract(range_t<8, 15>{}, pulses, offset_t<24>{}, std::integral_constant<bool, (N > 16) >{}); // no constepr-if in c++11
            //                if constexpr(N > 16) {
            //                    extract(range_t<8, 15>{}, pulses, offset_t<24>{});
            //                }
                            break;
                        case Frame::Ch1to16:
                            extract(range_t<0, 15>{}, pulses);
                            break;
                        case Frame::Ch1to12and64Switches:
                            extract(range_t<0, 11>{}, pulses);
                            sumSwitches();
                            break;
                        case Frame::Undefined:
                            break;
                        }
                    }
                    static inline uint16_t packages() {
                        return mPackagesCounter;
                    }
                    static inline uint16_t getbytes() {
                        return mBytesCounter;
                    }
                    static inline bool hasCommand() {
                        return mCommand != Command_t{};
                    }
                    static inline Command_t command() {
                        Command_t c{};
                        using std::swap;
                        swap(c, mCommand);
                        return c;
                    }
                private:
                    // uses tag-dispatch because no constexpr-if in c++11
                    template<uint8_t Begin, uint8_t End, uint8_t N, uint8_t Off = 0>
                    static inline void extract(const range_t<Begin, End>&, int16_t (&dest)[N], offset_t<Off> = offset_t<0>{}, std::true_type = std::true_type{}) {
                        static_assert((End - Begin) < (N - Off), "wrong range or target size");
                        uint8_t out{Off};
                        for(uint8_t i = Begin; i <= End; ++i) {
                            uint16_t raw = (sumdFrame[i].first << 8) | sumdFrame[i].second;
                            dest[out++] = convertSumdToPuls(raw);
                        }
                    }
                    template<uint8_t Begin, uint8_t End, uint8_t N, uint8_t Off = 0>
                    static inline void extract(const range_t<Begin, End>&, int16_t (&dest)[N], offset_t<Off>, std::false_type) {
                    }
                    static inline void sumSwitches() {
                        uint64_t sw = sumdFrame[12].first;
                        sw = (sw << 8) | sumdFrame[12].second;
                        sw = (sw << 8) | sumdFrame[13].first;
                        sw = (sw << 8) | sumdFrame[13].second;
                        sw = (sw << 8) | sumdFrame[14].first;
                        sw = (sw << 8) | sumdFrame[14].second;
                        sw = (sw << 8) | sumdFrame[15].first;
                        sw = (sw << 8) | sumdFrame[15].second;

                        uint64_t diff = lastSwitches ^ sw;

                        // for (uint8_t i = 0; i < MAX_LOGICAL_SWITCHES; ++i) {
                        //     const uint64_t mask = (((uint64_t)0x01) << i);
                        //     if (diff & mask) {
                        //         if (sw & mask) {
                        //             rawSetUnconnectedStickySwitch(i, true);
                        //         }
                        //         else {
                        //             rawSetUnconnectedStickySwitch(i, false);
                        //         }
                        //     }
                        // }
                        lastSwitches = sw;
                    }
                    static inline int16_t convertSumdToPuls(uint16_t const value) {
                        const int32_t centered = value - SumDV3::CenterValue;
                        // return std::clamp(((Trainer::MaxValue - Trainer::MinValue) * centered) / (SumDV3::MaxValue -SumDV3::MinValue));
                        return 0;
                    }
                    static uint8_t nChannels;
                    static Crc16 csum;
                    static uint8_t crcH;
                    static State mState;
                    static MesgType sumdFrame;
                    static uint8_t mIndex;
                    static uint16_t mPackagesCounter;
                    static uint16_t mBytesCounter;
                    static Frame frame;
                    static uint8_t reserved;
                    static uint8_t mode_cmd;
                    static uint8_t sub_cmd;
                    static uint64_t lastSwitches;
                    static Command_t mCommand;
                };
                // inline static member definitions not until c++17
                template<uint8_t Instance>
                uint8_t Servo<Instance>::nChannels{};
                template<uint8_t Instance>
                Crc16 Servo<Instance>::csum{};
                template<uint8_t Instance>
                uint8_t Servo<Instance>::crcH{};

                template<uint8_t Instance>
                typename Servo<Instance>::State Servo<Instance>::mState{Servo::State::Undefined};
                template<uint8_t Instance>
                typename Servo<Instance>::MesgType Servo<Instance>::sumdFrame;
                template<uint8_t Instance>
                uint8_t Servo<Instance>::mIndex{};

                template<uint8_t Instance>
                uint16_t Servo<Instance>::mPackagesCounter{};
                template<uint8_t Instance>
                uint16_t Servo<Instance>::mBytesCounter{};

                template<uint8_t Instance>
                typename Servo<Instance>::Frame Servo<Instance>::frame{Servo::Frame::Undefined};
                template<uint8_t Instance>
                uint8_t Servo<Instance>::reserved{};
                template<uint8_t Instance>
                uint8_t Servo<Instance>::mode_cmd{};
                template<uint8_t Instance>
                uint8_t Servo<Instance>::sub_cmd{};
                template<uint8_t Instance>
                uint64_t Servo<Instance>::lastSwitches{};
                template<uint8_t Instance>
                typename Servo<Instance>::Command_t Servo<Instance>::mCommand{};

            }
        }
        
        namespace SPort {
            // https://github.com/jcheger/frsky-arduino/tree/master/FrskySP
            // https://github.com/opentx/opentx/blob/2.3/radio/src/telemetry/frsky.h
            using namespace std::literals::chrono_literals;
            using namespace Units::literals;
            using namespace etl::literals;
            
            enum class SensorId : uint8_t { ID1  = 0x00, ID2  = 0xA1, ID3  = 0x22, ID4  = 0x83, ID5  = 0xE4, ID6  = 0x45, ID7  = 0xC6,
                                            ID8  = 0x67, ID9  = 0x48, ID10 = 0xE9, ID11 = 0x6A, ID12 = 0xCB, ID13 = 0xAC, ID14 = 0x0D,
                                            ID15 = 0x8E, ID16 = 0x2F, ID17 = 0xD0, ID18 = 0x71, ID19 = 0xF2, ID20 = 0x53, ID21 = 0x34,
                                            ID22 = 0x95, ID23 = 0x16, ID24 = 0xB7, ID25 = 0x98, ID26 = 0x39, ID27 = 0xBA, ID28 = 0x1B, ID_IGNORE = 0xFF };
            
            static inline constexpr std::array sensor_ids{SensorId::ID1, SensorId::ID2, SensorId::ID3, SensorId::ID4, SensorId::ID5, SensorId::ID6,
                                           SensorId::ID7, SensorId::ID8, SensorId::ID9, SensorId::ID10, SensorId::ID11, SensorId::ID12,
                                           SensorId::ID13, SensorId::ID14, SensorId::ID15, SensorId::ID16, SensorId::ID17, SensorId::ID18,
                                           SensorId::ID19, SensorId::ID20, SensorId::ID21, SensorId::ID22, SensorId::ID23, SensorId::ID24,
                                           SensorId::ID25, SensorId::ID26, SensorId::ID27, SensorId::ID28};
            
            static inline constexpr SensorId idFromIndex(const uint8_t i) {
                if (i < sensor_ids.size()) {
                    return sensor_ids[i];
                }
                return SensorId::ID3;
            }
            
            namespace detail {
                static_assert(etl::isSet(sensor_ids), "sensor ids not unique");
            }
            
            enum class ValueId : uint16_t {
                Current = 0x0200, // 15: 0,1A
                Voltage = 0x0210, // 15: 0,01V
                Cells   = 0x0300, // 15: Cellformat
                Temp1   = 0x0400, // 15: °C
                Temp2   = 0x0410, // 15: °C
                Rpm     = 0x0500, // 15: Rpm
                Fuel    = 0x0600, // 15: 0-100%
                Speed   = 0x0830, // 15: Knoten 0,001Kn
                State   = 0x0B20, // 15: RBox-State
                DIY     = 0x5100, // 255: DIY (State Info)
                DIY2    = 0x5200, // 255: DIY (State Info)
            };
    
            template<SensorId ID, template<typename> typename Uart, typename Timer, typename ProviderList = Meta::List<>>
            struct Sensor;
    
            template<SensorId ID, template<typename> typename Uart, typename Timer, typename... Providers>
            struct Sensor<ID, Uart, Timer, Meta::List<Providers...>> {
                //            inline static constexpr External::Tick<Timer> mResponseDelay{10};
                //            inline static constexpr External::Tick<Timer> mResponseDelay{2_ms};
                //                        std::integral_constant<uint16_t, mResponseDelay.value>::_;
                //            static_assert(mResponseDelay.value > 0);
                
                using providerList = Meta::List<Providers...>;
                inline static constexpr auto numberOfProviders = sizeof...(Providers);
                //            std::integral_constant<uint8_t, numberOfProviders>::_;
                
                
                static_assert(numberOfProviders > 0, "need at least one provider");
                
                using index_type = etl::ranged_circular<0, numberOfProviders - 1>;
                
                enum class State : uint8_t {Init, Request, ReplyWait, Reply, WaitReplyComplete};
                
//                inline static void maxProvider(uint8_t) {
//                }
                
                struct ProtocollAdapter {
                    //                using requests_t = std::conditional_t<uart::useInterrupts, volatile uint8_t, uint8_t>;
                    //                using requests_t = volatile uint8_t;
                    using requests_t = uint16_t;
                    
                    static inline bool process(const std::byte b) {
                        switch(mState) {
                        case State::Init: 
                            if (b == 0x7e_B) {
                                mState = State::Request;
                            }
                            break;
                        case State::Request: 
                            if (b == mPhysId) {
                                mRequests = mRequests + 1;
                                mState = State::ReplyWait;
                            }
                            else {
                                mState = State::Init;
                            }
                            break;
                        case State::ReplyWait:
                        case State::Reply:
                        case State::WaitReplyComplete:
                        default:
                            break;
                        }
                        return true;
                    }
                    inline static void id(const SensorId v) {
                        mPhysId = std::byte(v);
                    }
                    inline static void ratePeriodic() {}
                    
                    inline static auto requests() {
                        return mRequests;
                    }
                private:
                    static inline std::byte mPhysId{std::byte(SensorId::ID3)};
                    static inline requests_t mRequests{};
                };
                
                using uart = Uart<ProtocollAdapter>;   
                static_assert(uart::QueueLength >= 16);
                
                inline static constexpr bool useInterrupts = uart::useInterrupts;
//                using tick_type = std::conditional_t<useInterrupts, volatile External::Tick<Timer>, External::Tick<Timer>>;
                using state_type = std::conditional_t<useInterrupts, volatile State, State>;
                
                static inline void init() {
//                    if constexpr(std::is_same_v<typename uart::component_type, void>) {
//                        uart::template init<AVR::HalfDuplex>();
//                    }
//                    else {
//                        uart::template init<AVR::BaudRate<57600>, AVR::FullDuplex, false>(); // no pullup
//                        uart::rxInvert(true); // SPort Protocoll
//                    }
//                    uart::baud(57600);
                }
                template<bool B = true>
                static inline void enable() {
                    uart::template rxEnable<B>();
                }
                
                static inline void ratePeriodic() {
                }
                
                static inline void periodic() {
                    uart::periodic();
                    switch(mState) {
                    case State::ReplyWait:
                        mState = State::Reply;    
                        uart::template rxEnable<false>();
                        break;
                    case State::Reply:
                        reply();
                        mState = State::WaitReplyComplete;
                        break;
                    case State::WaitReplyComplete:
                        if (uart::isIdle()) {
                            ++mReplies;
                            uart::template rxEnable<true>();
                            mState = State::Init;
                        }
                        break;
                    case State::Init:
                    case State::Request:
                    default:
                        break;
                    }
                }
                static uint16_t replies() {
                    return mReplies;
                }
            private:
                static inline uint16_t mReplies{};
                struct CheckSum {
                    void operator+=(const std::byte b) {
                        mValue += uint8_t(b);
                        mValue += mValue >> 8;
                        mValue &= 0x00ff;
                    }
                    std::byte value() const {
                        return etl::nth_byte<0>(0xff - mValue);
                    }
                private:
                    uint16_t mValue{};
                };
                inline static void reply() {
                    CheckSum cs;
                    stuffResponse(0x10_B, cs);
                    Meta::visitAt<providerList>(mActualProvider, [&]<typename P>(Meta::Wrapper<P>){
                                                    auto id = ValueId(uint16_t(P::valueId) + mActualProvider);
                                                    stuff(id, cs);
                                                    stuff(P::value(), cs);
                                                });
                    stuff(cs);
                    ++mActualProvider;
                }
                inline static void stuff(const CheckSum& cs) {
                    uart::put(cs.value());
                }
                inline static void stuff(const uint32_t b, CheckSum& cs) {
                    stuffResponse(etl::nth_byte<0>(b), cs);
                    stuffResponse(etl::nth_byte<1>(b), cs);
                    stuffResponse(etl::nth_byte<2>(b), cs);
                    stuffResponse(etl::nth_byte<3>(b), cs);
                }
                inline static void stuff(const ValueId b, CheckSum& cs) {
                    using t = std::underlying_type<ValueId>::type;
                    stuffResponse(etl::nth_byte<0>(static_cast<t>(b)), cs);
                    stuffResponse(etl::nth_byte<1>(static_cast<t>(b)), cs);
                }
                inline static void stuffResponse_old(const std::byte b, CheckSum& cs) {
                    if (b == 0x7e_B) {
                        cs += 0x7d_B;
                        uart::put(0x7d_B);
                        cs += 0x5e_B;
                        uart::put(0x5e_B);
                    }
                    else if (b == 0x7d_B) {
                        cs += 0x7d_B;
                        uart::put(0x7d_B);
                        cs += 0x5d_B;
                        uart::put(0x5d_B);
                    }
                    else {
                        cs += b;
                        uart::put(b);
                    }
                }
                inline static void stuffResponse(const std::byte b, CheckSum& cs) {
                    cs += b;
                    if (b == 0x7e_B) {
                        uart::put(0x7d_B);
                        uart::put(0x5e_B);
                    }
                    else if (b == 0x7d_B) {
                        uart::put(0x7d_B);
                        uart::put(0x5d_B);
                    }
                    else {
                        uart::put(b);
                    }
                }
            private:            
                inline static index_type mActualProvider;
                inline static state_type mState{State::Init};
            };
        }
        
    }    
}
