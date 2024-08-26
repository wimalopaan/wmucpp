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
#include <type_traits>

#include "rc.h"
#include "crc.h"
#include "etl/fixedvector.h"
#include "tick.h"

namespace RC {
    namespace Protokoll {
        
        // https://github.com/crsf-wg/crsf/wiki
        
        namespace Crsf {
            /*
             * Richting: Handset (EdgeTx) -> TX-Module (ELRS) -> RX -> CC
             * SyncByte von EdgeTx ist immer: 0xee
             * nur ext-dest 0xc8 (FC) wird transportiert
             * -----------
             * Richtung: CC -> RX (ELRS) -> TX -> Handset (EdgeTx)
             * SyncByte (CC): 0xc8, 0xec, 0xea, geht nicht: 0xee, 0x00
             * ext dest: 0xea, 0xee, 0xec, 0xc8, auch Broadcast (0x00)
             * SyncByte (Telemetry-Mirror): immer 0xea (nicht 0xc8)
            */
            
            /*
             * sonstige Messages [dst, src]:
             * type: 0x3a [0xea, 0xee] : Radio-ID
             * type: 0x14 : link 
             * 
             */
            
            namespace Address {
                inline static constexpr std::byte StartByte{0xc8};
                inline static constexpr std::byte Broadcast{0x00};
                inline static constexpr std::byte Controller{0xc8};
                inline static constexpr std::byte Handset{0xea};
                inline static constexpr std::byte RX{0xec};
                inline static constexpr std::byte TX{0xee};
            }
            namespace Type {
                // https://github.com/EdgeTX/edgetx/blob/main/radio/src/telemetry/crossfire.h
                // simple
                inline static constexpr std::byte Gps{0x02};
                inline static constexpr std::byte Vario{0x07};
                inline static constexpr std::byte Battery{0x08};
                inline static constexpr std::byte Baro{0x09};
                inline static constexpr std::byte HeartBeat{0x0b};
                inline static constexpr std::byte Link{0x14};
                inline static constexpr std::byte Channels{0x16};
                inline static constexpr std::byte SubsetChannels{0x17};
                inline static constexpr std::byte LinkStatsRx{0x1c};
                inline static constexpr std::byte LinkStatsTx{0x1d};
                inline static constexpr std::byte Altitude{0x1e};
                inline static constexpr std::byte FlightMode{0x1e};
                // extended
                inline static constexpr std::byte Ping{0x28};
                inline static constexpr std::byte Info{0x29};
                inline static constexpr std::byte ParamEntry{0x2b};
                inline static constexpr std::byte ParamRead{0x2c};
                inline static constexpr std::byte ParamWrite{0x2d};
                inline static constexpr std::byte Command{0x32};

                inline static constexpr std::byte Custom1{0x40}; // pack type / instance-nr in payload
                inline static constexpr std::byte Custom2{0x41}; // pack type / instance-nr in payload
                inline static constexpr std::byte Custom3{0x42}; // pack type / instance-nr in payload
                inline static constexpr std::byte Custom4{0x43}; // pack type / instance-nr in payload
                inline static constexpr std::byte Temp1{0x48}; 
                inline static constexpr std::byte Temp2{0x49}; 
                inline static constexpr std::byte Temp3{0x4a}; 
                inline static constexpr std::byte Temp4{0x4b};
                //inline static constexpr std::byte Rpm1{0x78};
                inline static constexpr std::byte Rpm1{0x4c};
                inline static constexpr std::byte Rpm2{0x4d};
                inline static constexpr std::byte Rpm3{0x4e};
                inline static constexpr std::byte Rpm4{0x4f};

                inline static constexpr std::byte KissReq{0x78};
                inline static constexpr std::byte KissResp{0x79};

                inline static constexpr std::byte MspReq{0x7a};
                inline static constexpr std::byte MspResp{0x7b};
                inline static constexpr std::byte MspWrite{0x7c};

                inline static constexpr std::byte DisplayPort{0x7d};
            }
            namespace CommandType {
                // inline static constexpr std::byte bind{0x01}; // bind
                inline static constexpr std::byte rx{0x10}; // receiver command
                inline static constexpr std::byte general{0x0a}; // general command
                inline static constexpr std::byte CC{0xa0}; // CruiseController
                inline static constexpr std::byte Switch{0xa1}; // CruiseController
            }
            namespace CrsfCommand {
                inline static constexpr std::byte SelectID{0x05};
            }
            namespace SwitchCommand {
                inline static constexpr std::byte Set{0x01};
            }
            namespace CcCommand {
                inline static constexpr std::byte SetAltData{0x01}; // Index: [0, 255], value 8bit
                inline static constexpr std::byte SetAltChunk{0x02};
                inline static constexpr std::byte SetChannel{0x03}; // Index: [0, 63], value 16bit
            }
            namespace PacketIndex {
                inline static constexpr uint8_t address = 0;
                inline static constexpr uint8_t sync = 0;
                inline static constexpr uint8_t length = 1;
                inline static constexpr uint8_t type = 2;
            }
            static inline constexpr uint8_t  maxMessageSize = 64;
            static inline constexpr uint8_t  maxPayloadSize = 60;
            static inline constexpr uint8_t  maxExtendedPayloadSize = 58;
            
            static inline constexpr uint8_t  ValueBits = 11;
            static inline constexpr uint16_t ValueMask = ((1 << ValueBits) - 1);
            
            static inline constexpr uint16_t CenterValue = 0x3e0; // same center and range as sbus    

            using namespace std::literals::chrono_literals;
            using namespace etl::literals;

            namespace Lua {
                enum class CmdStep : uint8_t {
                    Idle = 0,
                    Click = 1,       // user has clicked the command to execute
                    Executing = 2,   // command is executing
                    AskConfirm = 3,  // command pending user OK
                    Confirmed = 4,   // user has confirmed
                    Cancel = 5,      // user has requested cancel
                    Query = 6,       // UI is requesting status update
                };
            }            
            template<typename T = uint8_t>
            struct Parameter {
                enum Type {U8 = 0, I8, U16, I16, F32 = 8, Sel, Str, Folder, Info, Command};
                uint8_t mParent{};
                Type mType = Type::U8;
                const char* mName{};
                const char* mOptions{};
                T* value_ptr = nullptr;
                T value() const {
                    if (value_ptr) {
                        return *value_ptr;
                    }
                    else {
                        return mValue;
                    }
                }
                void value(const T v) {
                    if (value_ptr) {
                        *value_ptr = v;
                    }
                    else {
                        mValue = v;
                    }
                }
                T mMinimum{};
                T mMaximum{};
                bool (*cb)(T){nullptr};
                T mDefault{mMinimum};
                T mUnits{0};
                T mValue{mDefault};
            };

#ifdef USE_CRSF_V2
            inline
#endif
            namespace V2 {

                struct __attribute__((packed)) Channels {
                    unsigned ch0 : 11;
                    unsigned ch1 : 11;
                    unsigned ch2 : 11;
                    unsigned ch3 : 11;
                    unsigned ch4 : 11;
                    unsigned ch5 : 11;
                    unsigned ch6 : 11;
                    unsigned ch7 : 11;
                    unsigned ch8 : 11;
                    unsigned ch9 : 11;
                    unsigned ch10 : 11;
                    unsigned ch11 : 11;
                    unsigned ch12 : 11;
                    unsigned ch13 : 11;
                    unsigned ch14 : 11;
                    unsigned ch15 : 11;
                };

                template<typename UART, typename SystemTimer, typename MCU = DefaultMcu>
                struct Generator {
                    using timer = SystemTimer;
                    using uart = UART;
                    static inline constexpr External::Tick<timer> crsfTicks{2ms};

                    static inline void target(const std::byte t) {
                        mTarget = t;
                    }

                    template<auto N>
                    requires(N >= 16)
                    static inline void channels(const std::array<uint16_t, N>& channels) {
                        Channels packed;
                        // packed.ch0 = toCrsfRange(channels[0]);
                        packed.ch1 = channels[1];
                        packed.ch2 = channels[2];
                        packed.ch3 = channels[3];
                        packed.ch4 = channels[4];
                        packed.ch5 = channels[5];
                        packed.ch6 = channels[6];
                        packed.ch7 = channels[7];
                        packed.ch8 = channels[8];
                        packed.ch9 = channels[9];
                        packed.ch10 = channels[10];
                        packed.ch11 = channels[11];
                        packed.ch12 = channels[12];
                        packed.ch13 = channels[13];
                        packed.ch14 = channels[14];
                        packed.ch15 = channels[15];

                        const std::byte* ptr = (std::byte*)&packed;

                        CRC8 crc;
                        mData.clear();
                        mData.push_back(mTarget);
                        mData.push_back(std::byte(22 + 2));
                        mData.push_back(Type::Channels);
                        crc += Type::Channels;
                        for(uint8_t i = 0; i < 22; ++i) {
                            crc += ptr[i];
                            mData.push_back(ptr[i]);
                        }
                        mData.push_back(crc);
                        mState = State::Wait;
                    }

                    // static inline void channels(const Channels& channels) {
                    // }

                    template<typename T>
                    static inline void data(const std::byte type, const T& values) {
                        CRC8 crc;
                        mData.clear();
                        mData.push_back(mTarget);
                        mData.push_back(std::byte(values.size() + 2));
                        mData.push_back(type);
                        crc += type;
                        for(const auto& v: values) {
                            crc += v;
                            mData.push_back(v);
                        }
                        mData.push_back(crc);
                        mState = State::Wait;
                    }

                    enum class State : uint8_t {Idle, Wait, Sending};

                    static inline void clear() {
                        uart::clear();
                    }

                    static inline void ratePeriodic() {
                        switch(mState) {
                        case State::Idle:
                            break;
                        case State::Wait:
                            if (uart::isTxQueueEmpty()) {
                                mState = State::Sending;
                            }
                            break;
                        case State::Sending:
                            for(const auto& b: mData) {
                                uart::put(b);
                            }
                            mState = State::Idle;
                            break;
                        }
                    }
                    private:
                    static inline std::byte mTarget{0xc8_B};
                    static inline State mState{State::Idle};
                    static inline etl::FixedVector<std::byte, maxMessageSize> mData;
                };

                template<uint8_t N, typename Config, typename MCU = DefaultMcu>
                struct Adapter {
                    using debug = Config::dbg;
                    using out = Config::out;
                    using CB = Config::callback;
                    using systemTimer = Config::timer;

                    static inline External::Tick<systemTimer> mPauseTick;
                    static inline constexpr External::Tick<systemTimer> mPauseTicks{100ms};

                    static inline constexpr uint8_t CRSFChannels{16};

                    enum class State : uint8_t {Undefined,
                                                GotStart, GotLength,
                                                Link, Channels, Ping, Info, ParameterEntry, ParameterRead, ParameterWrite, Data, Command};

                    struct Responder {
                        static inline void setExtendedDestination(const std::byte d) {
                            if (d != std::byte{0x00}) {
                                mExtendedDestination = d;
                            }
                            else {
                                mExtendedDestination = RC::Protokoll::Crsf::Address::Handset;
                            }
                        }

                        static inline void sendDeviceInfo() {
                            if (!mEnableReply) return;
                            using namespace etl::literals;
                            IO::outl<debug>("# DI adr: ", (uint8_t)mExtendedDestination, " src: ", (uint8_t)mExtendedSource);
                            mReply.clear();
                            mReply.push_back(mExtendedDestination);
                            mReply.push_back(mExtendedSource);
                            etl::push_back_ntbs(CB::name(), mReply);
                            etl::serialize(CB::serialNumber(), mReply);
                            etl::serialize(CB::hwVersion(), mReply);
                            etl::serialize(CB::swVersion(), mReply);
                            etl::serialize(CB::numberOfParameters(), mReply);
                            etl::serialize(CB::protocolVersion(), mReply);
                            out::data(RC::Protokoll::Crsf::Type::Info, mReply);
                        }

                        static inline void sendCommandResponse(const uint8_t pIndex, const uint8_t value) {
                            if (!mEnableReply) return;
                            IO::outl<debug>("# sCR adr: ", (uint8_t)mExtendedDestination, " i: ", pIndex, " v: ", value);
                            mReply.clear();
                            mReply.push_back(mExtendedDestination);
                            mReply.push_back(mExtendedSource);
                            etl::serialize(pIndex, mReply);
                            etl::serialize(uint8_t{}, mReply); // remaining chunks
                            etl::serialize(CB::parameter(pIndex).mParent, mReply); // parent
                            etl::serialize((uint8_t)CB::parameter(pIndex).mType, mReply); // field type
                            if (CB::parameter(pIndex).mName) {
                                etl::push_back_ntbs(CB::parameter(pIndex).mName, mReply);
                            }
                            if (value == 1) { // start
                                if (const auto cb = CB::parameter(pIndex).cb; cb) {
                                    if (cb(1)) {
                                        etl::serialize(uint8_t{3}, mReply); // ask confirm
                                    }
                                    else {
                                        etl::serialize(uint8_t{2}, mReply); // executing
                                    }
                                }
                                etl::serialize(uint8_t{100}, mReply); // update-timeout: 1s
                                if (CB::parameter(pIndex).mOptions) {
                                    etl::push_back_ntbs(CB::parameter(pIndex).mOptions, mReply);
                                }
                                else {
                                    etl::serialize(uint8_t{0}, mReply);
                                }
                            }
                            else if (value == 4) { // confirmed
                                if (const auto cb = CB::parameter(pIndex).cb; cb) {
                                    cb(4);
                                }
                                etl::serialize(uint8_t{2}, mReply); // executing
                                etl::serialize(uint8_t{100}, mReply); // update-timeout: 1s
                                if (CB::parameter(pIndex).mOptions) {
                                    etl::push_back_ntbs(CB::parameter(pIndex).mOptions, mReply);
                                }
                                else {
                                    etl::serialize(uint8_t{0}, mReply);
                                }
                            }
                            else if (value == 5) { // cancel
                                if (const auto cb = CB::parameter(pIndex).cb; cb) {
                                    cb(5);
                                }
                            }
                            else if (value == 6) { // update-request
                                if (const auto cb = CB::parameter(pIndex).cb; cb) {
                                    const bool res = cb(6);
                                    if (res) { // still executing
                                        etl::serialize(uint8_t{2}, mReply);
                                    }
                                    else {
                                        etl::serialize(uint8_t{0}, mReply); // finished
                                    }
                                }
                                else {
                                    etl::serialize(uint8_t{0}, mReply); // finished
                                }
                                etl::serialize(uint8_t{100}, mReply); // update-timeout: 1s

                                if (CB::parameter(pIndex).mOptions) {
                                    IO::outl<debug>("# sCR opt: ", CB::parameter(pIndex).mOptions);
                                    etl::push_back_ntbs(CB::parameter(pIndex).mOptions, mReply);
                                }
                                else {
                                    etl::serialize(uint8_t{0}, mReply);
                                }
                            }
                            out::data(RC::Protokoll::Crsf::Type::ParamEntry, mReply);
                        }
                        static inline void sendParameterInfo(const uint8_t pIndex) {
                            if (!mEnableReply) return;
                            IO::outl<debug>("# PI adr: ", (uint8_t)mExtendedDestination, " i: ", pIndex);
                            mReply.clear();
                            mReply.push_back(mExtendedDestination);
                            mReply.push_back(mExtendedSource);
                            etl::serialize(pIndex, mReply);
                            etl::serialize(uint8_t{}, mReply); // remaining chunks
                            etl::serialize(CB::parameter(pIndex).mParent, mReply); // parent
                            etl::serialize((uint8_t)CB::parameter(pIndex).mType, mReply); // field type
                            if (CB::parameter(pIndex).mName) {
                                etl::push_back_ntbs(CB::parameter(pIndex).mName, mReply);
                            }
                            if ((CB::parameter(pIndex).mType != 0x0d) && CB::parameter(pIndex).mOptions) {
                                etl::push_back_ntbs(CB::parameter(pIndex).mOptions, mReply);
                            }
                            if (CB::parameter(pIndex).mType < 0x0a) {
                                etl::serialize(CB::parameter(pIndex).value(), mReply); // value
                                etl::serialize(CB::parameter(pIndex).mMinimum, mReply);
                                etl::serialize(CB::parameter(pIndex).mMaximum, mReply);
                                etl::serialize(CB::parameter(pIndex).mDefault, mReply);
                                etl::serialize(CB::parameter(pIndex).mUnits, mReply);
                            }
                            if (CB::parameter(pIndex).mType == 0x0a) { // string
                                etl::serialize(CB::parameter(pIndex).mUnits, mReply);
                            }
                            if (CB::parameter(pIndex).mType == 0x0d) {
                                etl::serialize(CB::parameter(pIndex).value(), mReply);
                                etl::serialize((uint8_t)0x00, mReply);
                                etl::serialize((uint8_t)0xc8, mReply);
                                etl::serialize((uint8_t)0x00, mReply);
                            }
                            out::data(RC::Protokoll::Crsf::Type::ParamEntry, mReply);
                        }
                        static inline std::byte address() {
                            return mExtendedSource;
                        }
                        static inline void address(const std::byte adr) {
                            mExtendedSource = adr;
                        }
                        private:
                        static inline etl::FixedVector<std::byte, RC::Protokoll::Crsf::maxPayloadSize> mReply;
                        static inline std::byte mExtendedDestination{};
                        static inline std::byte mExtendedSource{RC::Protokoll::Crsf::Address::Controller};
                    };

                    static inline void ratePeriodic(const auto f) {
                        (++mPauseTick).on(mPauseTicks, [&]{
                            mBytesCounter = 0;
                            mPackagesCounter = 0;
                            mCommandPackagesCounter = 0;
                            f(false);
                        });
                        if (mPackagesCounter > 10) {
                            f(true);
                        }
                    }

                    inline static void process(const std::byte b) {
                        mPauseTick.reset();
                        ++mBytesCounter;

                        switch(mState) { // enum-switch -> no default (intentional)
                        case State::Undefined:
                            csum.reset();
                            if (b == Crsf::Address::StartByte){
                                mAddress = b;
                                mState = State::GotStart;
                            }
                            break;
                        case State::GotStart:
                            if (const uint8_t l = static_cast<uint8_t>(b); (l > 2) && (l <= mData.size())) {
                                mLength = l - 2; // only payload (not including type and crc)
                                mPayloadIndex = 0;
                                mState = State::GotLength;
                            }
                            else {
                                mState = State::Undefined;
                            }
                            break;
                        case State::GotLength:
                            csum += b;
                            if (b == Crsf::Type::Link) {
                                mState = State::Link;
                            }
                            else if (b == Crsf::Type::Channels) {
                                mState = State::Channels;
                            }
                            else if (b == Crsf::Type::Ping) {
                                mState = State::Ping;
                            }
                            else if (b == Crsf::Type::Info) {
                                mState = State::Info;
                            }
                            else if (b == Crsf::Type::ParamEntry) {
                                mState = State::ParameterEntry;
                            }
                            else if (b == Crsf::Type::ParamRead) {
                                mState = State::ParameterRead;
                            }
                            else if (b == Crsf::Type::ParamWrite) {
                                mState = State::ParameterWrite;
                            }
                            else if (b == Crsf::Type::Command) {
                                mState = State::Command;
                            }
                            else {
                                mState = State::Data;
                            }
                            break;
                        case State::Link:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mLinkPackagesCounter;
                                    ++mPackagesCounter;
                                    CB::gotLinkStats();
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                ++mPayloadIndex;
                            }
                            break;
                        case State::Channels:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mChannelsPackagesCounter;
                                    ++mPackagesCounter;
                                    convert();
                                    CB::gotChannels();
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                mData[mPayloadIndex] = static_cast<uint8_t>(b);
                                ++mPayloadIndex;
                            }
                            break;
                        case State::Ping:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mPingPackagesCounter;
                                    ++mPackagesCounter;
                                    Responder::sendDeviceInfo();
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                if (mPayloadIndex == 0) {
                                    mExtendedDestination = b;
                                }
                                else if (mPayloadIndex == 1) {
                                    Responder::setExtendedDestination(b);
                                    mExtendedSource = b;
                                }
                                else {
                                }
                                ++mPayloadIndex;
                            }
                            break;
                        case State::Info:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mPackagesCounter;
                                    ++mInfoPackagesCounter;
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                ++mPayloadIndex;
                            }
                            break;
                        case State::ParameterEntry:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mParameterEntryPackagesCounter;
                                    ++mPackagesCounter;
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                ++mPayloadIndex;
                            }
                            break;
                        case State::ParameterRead:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mPackagesCounter;
                                    IO::outl<debug>("# PR ", (uint8_t)mExtendedDestination, " i: ", mParameterIndex, " src: ", (uint8_t)mExtendedSource);
                                    if (mExtendedDestination == Responder::address()) {
                                        ++mParameterReadPackagesCounter;
                                        Responder::sendParameterInfo(mParameterIndex);
                                    }
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                if (mPayloadIndex == 0) {
                                    mExtendedDestination = b;
                                }
                                else if (mPayloadIndex == 1) {
                                    Responder::setExtendedDestination(b);
                                    mExtendedSource = b;
                                }
                                else if (mPayloadIndex == 2) {
                                    mParameterIndex = (uint8_t)b;
                                }
                                else if (mPayloadIndex == 3) {
                                    mParameterChunk = (uint8_t)b;
                                }
                                ++mPayloadIndex;
                            }
                            break;
                        case State::ParameterWrite:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mPackagesCounter;
                                    IO::outl<debug>("# PW ", (uint8_t)mExtendedDestination, " i: ", mParameterIndex, " src: ", (uint8_t)mExtendedSource);
                                    if (mExtendedDestination == Responder::address()) {
                                        ++mParameterWritePackagesCounter;
                                        if (CB::isCommand(mParameterIndex)) {
                                            Responder::sendCommandResponse(mParameterIndex, mParameterValue);
                                        }
                                        else {
                                            if (mEnableReply) {
                                                CB::setParameter(mParameterIndex, mParameterValue);
                                            }
                                        }
                                    }
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                if (mPayloadIndex == 0) {
                                    mExtendedDestination = b;
                                }
                                else if (mPayloadIndex == 1) {
                                    mExtendedSource = b;
                                }
                                else if (mPayloadIndex == 2) {
                                    mParameterIndex = (uint8_t)b;
                                }
                                else if (mPayloadIndex == 3) {
                                    mParameterValue = (uint8_t)b;
                                }
                                ++mPayloadIndex;
                            }
                            break;
                        case State::Command:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mPackagesCounter;
                                    IO::outl<debug>("# Co ", (uint8_t)mExtendedDestination, " i: ", mParameterIndex, " src: ", (uint8_t)mExtendedSource);
                                    if (mExtendedDestination == Responder::address()) {
                                        ++mCommandPackagesCounter;
                                        CB::command(mData);
                                    }
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                mData[mPayloadIndex] = static_cast<uint8_t>(b);
                                if (mPayloadIndex == 0) {
                                    mExtendedDestination = b;
                                }
                                else if (mPayloadIndex == 1) {
                                    mExtendedSource = b;
                                }
                                ++mPayloadIndex;
                            }
                            break;
                        case State::Data:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mPackagesCounter;
                                    IO::outl<debug>("# Da ", (uint8_t)mExtendedDestination, " i: ", mParameterIndex, " src: ", (uint8_t)mExtendedSource);
                                    if (mExtendedDestination == Responder::address()) {
                                        ++mDataPackagesCounter;
                                        // decode
                                    }
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                ++mPayloadIndex;
                            }
                            break;
                        }
                    }
                    static inline uint16_t packages() {
                        return mPackagesCounter;
                    }
                    static inline uint16_t getBytes() {
                        return mBytesCounter;
                    }
                    static inline const auto& values() {
                        return mChannels;
                    }

                    template<typename C>
                    static inline void copyChangedChannels(C& out) {
                        for(uint8_t i = 0; (i < mChannels.size()) && (i < out.size()); ++i) {
                            if (mChannels[i] != mPreviousChannelValues[i]) {
                                if ((mChannels[i] >= 172) && (mChannels[i] <= 1812)) {
                                    mPreviousChannelValues[i] = mChannels[i];
                                    out[i] = mChannels[i];
                                }
                            }
                        }
                    }

                    using normalized_type = etl::ranged_NaN<-1000, +1000>;

                    static inline normalized_type normalized(const uint8_t ch) {
                        if (ch < 16) {
                            if ((mChannels[ch] >= 172) && (mChannels[ch] <= 1812)) {
                                const float d = mChannels[ch] - 992;
                                const float nd = d * 1000.0f / 840.0f;
                                return normalized_type(nd);
                            }
                        }
                        return normalized_type{0};
                    }

                    static inline void enableReply(const bool on = true) {
                        mEnableReply = on;
                    }

                    //           private:
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
                    inline static bool mEnableReply{true};
                    inline static CRC8 csum;
                    inline static State mState{State::Undefined};
                    inline static std::array<uint8_t, maxMessageSize> mData;
                    inline static std::byte mAddress{};
                    inline static std::byte mExtendedDestination{};
                    inline static std::byte mExtendedSource{};
                    inline static uint8_t mParameterIndex{};
                    inline static uint8_t mParameterChunk{};
                    inline static uint8_t mParameterValue{};
                    inline static uint8_t mPayloadIndex{};
                    inline static uint8_t mLength{};
                    inline static uint16_t mPackagesCounter{};
                    inline static uint16_t mLinkPackagesCounter{};
                    inline static uint16_t mChannelsPackagesCounter{};
                    inline static uint16_t mPingPackagesCounter{};
                    inline static uint16_t mParameterEntryPackagesCounter{};
                    inline static uint16_t mParameterReadPackagesCounter{};
                    inline static uint16_t mParameterWritePackagesCounter{};
                    inline static uint16_t mCommandPackagesCounter{};
                    inline static uint16_t mInfoPackagesCounter{};
                    inline static uint16_t mDataPackagesCounter{};

                    inline static uint16_t mBytesCounter{};

                    using values_t = std::array<uint16_t, 16>;
                    inline static values_t mChannels;
                    inline static values_t mPreviousChannelValues;
                };
            }

            namespace V1 {
                template<uint8_t N, typename CB, typename Debug = void, typename MCU = DefaultMcu>
                struct Adapter {
                    using debug = Debug;
                    using out = CB::out;

                    static inline constexpr uint8_t mPauseCount{2}; // 2ms

                    static inline constexpr uint8_t CRSFChannels{16};

                    enum class State : uint8_t {Undefined,
                                                GotAddress, GotLength,
                                                Link, Channels, Ping, Info, ParameterEntry, ParameterRead, ParameterWrite, Data, Command};

    //                static inline void tick1ms() {
    //                    if (mPauseCounter > 0) {
    //                        --mPauseCounter;
    //                    }
    //                    else {
    //                        mState = State::Undefined;
    //                    }
    //                }

                    struct Responder {
                        static inline void setExtendedDestination(const std::byte d) {
                            if (d != std::byte{0x00}) {
                                mExtendedDestination = d;
                            }
                            else {
                                mExtendedDestination = RC::Protokoll::Crsf::Address::Handset;
                            }
                        }

                        static inline void sendDeviceInfo() {
                            using namespace etl::literals;
    //                        IO::outl<debug>("# DI adr: ", (uint8_t)mExtendedDestination);
                            mReply.clear();
                            mReply.push_back(mExtendedDestination);
                            mReply.push_back(mExtendedSource);
                            etl::push_back_ntbs(CB::name(), mReply);
                            etl::serialize(CB::serialNumber(), mReply);
                            etl::serialize(CB::hwVersion(), mReply);
                            etl::serialize(CB::swVersion(), mReply);
                            etl::serialize(CB::numberOfParameters(), mReply);
                            etl::serialize(CB::protocolVersion(), mReply);
                            out::data(RC::Protokoll::Crsf::Type::Info, mReply);
                        }

                        static inline void sendCommandResponse(const uint8_t pIndex, const uint8_t value) {
                            if (!std::is_same_v<debug, void>) {
                                IO::outl<debug>("# sCR adr: ", (uint8_t)mExtendedDestination, " i: ", pIndex);
                            }
                            mReply.clear();
                            mReply.push_back(mExtendedDestination);
                            mReply.push_back(mExtendedSource);
                            etl::serialize(pIndex, mReply);
                            etl::serialize(uint8_t{}, mReply); // remaining chunks
                            etl::serialize(CB::parameter(pIndex).mParent, mReply); // parent
                            etl::serialize((uint8_t)CB::parameter(pIndex).mType, mReply); // field type
                            if (CB::parameter(pIndex).mName) {
                                etl::push_back_ntbs(CB::parameter(pIndex).mName, mReply);
                            }
                            if (value == 1) {
                                etl::serialize(uint8_t{2}, mReply); // executing
                                etl::serialize(uint8_t{0xc8}, mReply);
                                if (CB::parameter(pIndex).mOptions) {
                                    etl::push_back_ntbs(CB::parameter(pIndex).mOptions, mReply);
                                }
                            }
                            if (value == 6) {
                                etl::serialize(uint8_t{0}, mReply);
                                etl::serialize(uint8_t{0xc8}, mReply);
                                etl::serialize(uint8_t{0}, mReply);
                            }
                            out::data(RC::Protokoll::Crsf::Type::ParamEntry, mReply);
                        }
                        static inline void sendParameterInfo(const uint8_t pIndex) {
    //                        IO::outl<debug>("# PI adr: ", (uint8_t)mExtendedDestination, " i: ", pIndex);
                            mReply.clear();
                            mReply.push_back(mExtendedDestination);
                            mReply.push_back(mExtendedSource);
                            etl::serialize(pIndex, mReply);
                            etl::serialize(uint8_t{}, mReply); // remaining chunks
                            etl::serialize(CB::parameter(pIndex).mParent, mReply); // parent
                            etl::serialize((uint8_t)CB::parameter(pIndex).mType, mReply); // field type
                            if (CB::parameter(pIndex).mName) {
                                etl::push_back_ntbs(CB::parameter(pIndex).mName, mReply);
                            }
                            if ((CB::parameter(pIndex).mType != 0x0d) && CB::parameter(pIndex).mOptions) {
                                etl::push_back_ntbs(CB::parameter(pIndex).mOptions, mReply);
                            }
                            if (CB::parameter(pIndex).mType <= 0x0a) {
                                etl::serialize(CB::parameter(pIndex).value(),mReply); // value
                                etl::serialize(CB::parameter(pIndex).mMinimum, mReply);
                                etl::serialize(CB::parameter(pIndex).mMaximum, mReply);
                                etl::serialize(CB::parameter(pIndex).mDefault, mReply);
                                etl::serialize(CB::parameter(pIndex).mUnits, mReply);
                            }
                            if (CB::parameter(pIndex).mType == 0x0d) {
                                etl::serialize(CB::parameter(pIndex).value(), mReply);
                                etl::serialize((uint8_t)0x00, mReply);
                                etl::serialize((uint8_t)0xc8, mReply);
                                etl::serialize((uint8_t)0x00, mReply);
                            }
                            out::data(RC::Protokoll::Crsf::Type::ParamEntry, mReply);
                        }
                    private:
                        static inline etl::FixedVector<std::byte, RC::Protokoll::Crsf::maxPayloadSize> mReply;
                        static inline std::byte mExtendedDestination{};
                        static inline std::byte mExtendedSource{RC::Protokoll::Crsf::Address::Controller};
                    };


                    inline static void process(const std::byte b) {
    //                    mPauseCounter = mPauseCount;
                        ++mBytesCounter;

                        switch(mState) { // enum-switch -> no default (intentional)
                        case State::Undefined:
                            csum.reset();
                            if ((b == Crsf::Address::Controller) ||
                                    (b == Crsf::Address::Handset) ||
                                    (b == Crsf::Address::RX) ||
                                    (b == Crsf::Address::TX) ||
                                    (b == Crsf::Address::Broadcast))
                            {
                                mAddress = b;
                                mState = State::GotAddress;
                            }
                        break;
                        case State::GotAddress:
                            if ((static_cast<uint8_t>(b) > 2) && (static_cast<uint8_t>(b) <= mData.size())) {
                                mLength = static_cast<uint8_t>(b) - 2; // only payload (not including type and crc)
                                mPayloadIndex = 0;
                                mState = State::GotLength;
                            }
                            else {
                                mState = State::Undefined;
                            }
                        break;
                        case State::GotLength:
                            csum += b;
                            if (b == Crsf::Type::Link) {
                                mState = State::Link;
                            }
                            else if (b == Crsf::Type::Channels) {
                                mState = State::Channels;
                            }
                            else if (b == Crsf::Type::Ping) {
                                mState = State::Ping;
                            }
                            else if (b == Crsf::Type::Info) {
                                mState = State::Info;
                            }
                            else if (b == Crsf::Type::ParamEntry) {
                                mState = State::ParameterEntry;
                            }
                            else if (b == Crsf::Type::ParamRead) {
                                mState = State::ParameterRead;
                            }
                            else if (b == Crsf::Type::ParamWrite) {
                                mState = State::ParameterWrite;
                            }
                            else if (b == Crsf::Type::Command) {
                                mState = State::Command;
                            }
                            else {
                                mState = State::Data;
                            }
                        break;
                        case State::Link:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mLinkPackagesCounter;
                                    ++mPackagesCounter;
                                    // decode
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                ++mPayloadIndex;
                            }
                        break;
                        case State::Channels:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mChannelsPackagesCounter;
                                    ++mPackagesCounter;
                                    convert();
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                mData[mPayloadIndex] = static_cast<uint8_t>(b);
                                ++mPayloadIndex;
                            }
                        break;
                        case State::Ping:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mPingPackagesCounter;
                                    ++mPackagesCounter;
                                    Responder::sendDeviceInfo();
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                if (mPayloadIndex == 0) {
                                    mExtendedDestination = b;
                                }
                                else if (mPayloadIndex == 1) {
                                    Responder::setExtendedDestination(b);
                                    mExtendedSource = b;
                                }
                                else {
                                }
                                ++mPayloadIndex;
                            }
                        break;
                        case State::Info:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mInfoPackagesCounter;
                                    ++mPackagesCounter;
                                    // decode
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                ++mPayloadIndex;
                            }
                        break;
                        case State::ParameterEntry:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mParameterEntryPackagesCounter;
                                    ++mPackagesCounter;
                                    // decode
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                ++mPayloadIndex;
                            }
                        break;
                        case State::ParameterRead:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mParameterReadPackagesCounter;
                                    ++mPackagesCounter;
                                    Responder::sendParameterInfo(mParameterIndex);
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                if (mPayloadIndex == 0) {
                                    mExtendedDestination = b;
                                }
                                else if (mPayloadIndex == 1) {
                                    Responder::setExtendedDestination(b);
                                    mExtendedSource = b;
                                }
                                else if (mPayloadIndex == 2) {
                                    mParameterIndex = (uint8_t)b;
                                }
                                else if (mPayloadIndex == 3) {
                                    mParameterChunk = (uint8_t)b;
                                }
                                ++mPayloadIndex;
                            }
                        break;
                        case State::ParameterWrite:
                            if (!std::is_same_v<debug, void>) {
                                IO::outl<debug>("# PW", (uint8_t)mExtendedDestination, " i: ", mParameterIndex);
                            }
                            if (mPayloadIndex >= mLength) {
                                if (!std::is_same_v<debug, void>) {
                                    IO::outl<debug>("#PW", (uint8_t)mExtendedDestination, " i: ", mParameterIndex);
                                }
                                if (csum == b) {
                                    ++mParameterWritePackagesCounter;
                                    ++mPackagesCounter;
                                    if (CB::isCommand(mParameterIndex)) {
                                        Responder::sendCommandResponse(mParameterIndex, mParameterValue);
                                    }
                                    else {
                                        CB::setParameter(mParameterIndex, mParameterValue);
                                    }
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                if (mPayloadIndex == 0) {
                                    mExtendedDestination = b;
                                }
                                else if (mPayloadIndex == 1) {
                                    mExtendedSource = b;
                                }
                                else if (mPayloadIndex == 2) {
                                    mParameterIndex = (uint8_t)b;
                                }
                                else if (mPayloadIndex == 3) {
                                    mParameterValue = (uint8_t)b;
                                }
                                ++mPayloadIndex;
                            }
                        break;
                        case State::Command:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mCommandPackagesCounter;
                                    ++mPackagesCounter;
                                    // decode
                                    CB::command(mData);
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                mData[mPayloadIndex] = static_cast<uint8_t>(b);
                                ++mPayloadIndex;
                            }
                        break;
                        case State::Data:
                            if (mPayloadIndex >= mLength) {
                                if (csum == b) {
                                    ++mDataPackagesCounter;
                                    ++mPackagesCounter;
                                    // decode
                                }
                                mState = State::Undefined;
                            }
                            else {
                                csum += b;
                                ++mPayloadIndex;
                            }
                        break;
                        }
                    }
                    static inline uint16_t packages() {
                        return mPackagesCounter;
                    }
                    static inline uint16_t getBytes() {
                        return mBytesCounter;
                    }
                    static inline const auto& values() {
                        return mChannels;
                    }

                    template<typename C>
                    static inline void copyChangedChannels(C& out) {
                        for(uint8_t i = 0; (i < mChannels.size()) && (i < out.size()); ++i) {
                            if (mChannels[i] != mPreviousChannelValues[i]) {
                                if ((mChannels[i] >= 172) && (mChannels[i] <= 1812)) {
                                    mPreviousChannelValues[i] = mChannels[i];
                                    out[i] = mChannels[i];
                                }
                            }
                        }
                    }

    //           private:
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
                    inline static CRC8 csum;
                    inline static State mState{State::Undefined};
                    inline static std::array<uint8_t, maxMessageSize> mData;
                    inline static std::byte mAddress{};
                    inline static std::byte mExtendedDestination{};
                    inline static std::byte mExtendedSource{};
                    inline static uint8_t mParameterIndex{};
                    inline static uint8_t mParameterChunk{};
                    inline static uint8_t mParameterValue{};
                    inline static uint8_t mPayloadIndex{};
                    inline static uint8_t mLength{};
                    inline static uint16_t mPackagesCounter{};
                    inline static uint16_t mLinkPackagesCounter{};
                    inline static uint16_t mChannelsPackagesCounter{};
                    inline static uint16_t mPingPackagesCounter{};
                    inline static uint16_t mParameterEntryPackagesCounter{};
                    inline static uint16_t mParameterReadPackagesCounter{};
                    inline static uint16_t mParameterWritePackagesCounter{};
                    inline static uint16_t mCommandPackagesCounter{};
                    inline static uint16_t mInfoPackagesCounter{};
                    inline static uint16_t mDataPackagesCounter{};

                    inline static uint16_t mBytesCounter{};
                    inline static uint8_t mPauseCounter{};

                    using values_t = std::array<uint16_t, 16>;
                    inline static values_t mChannels;
                    inline static values_t mPreviousChannelValues;
                };

            }
        }
    }
}
