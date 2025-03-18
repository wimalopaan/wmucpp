#pragma once

#include "crc.h"

namespace Crsf {
    static inline constexpr uint32_t baudrate{420'000};
    static inline constexpr std::array<uint32_t, 2> baudrates{420'000, 921'000};
    namespace Address {
        inline static constexpr std::byte StartByte{0xc8};
        inline static constexpr std::byte Broadcast{0x00};
        inline static constexpr std::byte Controller{0xc8};
        inline static constexpr std::byte Handset{0xea};
        inline static constexpr std::byte RX{0xec};
        inline static constexpr std::byte TX{0xee};
    }
    namespace Type {
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
        inline static constexpr std::byte FlightMode{0x21};
        // extended
        inline static constexpr std::byte Ping{0x28};
        inline static constexpr std::byte Info{0x29};
        inline static constexpr std::byte ParamEntry{0x2b};
        inline static constexpr std::byte ParamRead{0x2c};
        inline static constexpr std::byte ParamWrite{0x2d};
        inline static constexpr std::byte Command{0x32};
        inline static constexpr std::byte RadioID{0x3a};

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

        inline static constexpr std::byte Diy1{0x50};
        inline static constexpr std::byte Diy2{0x51};
        inline static constexpr std::byte Diy3{0x52};
        inline static constexpr std::byte Diy4{0x53};

        inline static constexpr std::byte KissReq{0x78};
        inline static constexpr std::byte KissResp{0x79};

        inline static constexpr std::byte MspReq{0x7a};
        inline static constexpr std::byte MspResp{0x7b};
        inline static constexpr std::byte MspWrite{0x7c};

        inline static constexpr std::byte DisplayPort{0x7d};

        inline static constexpr std::byte PassThru{0x7f};
        inline static constexpr std::byte ArduPilot{0x80};
    }
    namespace CommandType {
        // inline static constexpr std::byte bind{0x01}; // bind
        inline static constexpr std::byte rx{0x10}; // receiver command
        inline static constexpr std::byte general{0x0a}; // general command
        inline static constexpr std::byte CC{0xa0}; // CruiseController
        inline static constexpr std::byte Switch{0xa1}; // MultiSwitch
        inline static constexpr std::byte Schottel{0xa2}; // RC720E32
    }
    namespace SchottelCommand {
        inline static constexpr std::byte Reset{0x01};
    }
    namespace CrsfCommand {
        inline static constexpr std::byte SelectID{0x05};
    }
    namespace SwitchCommand {
        inline static constexpr std::byte Set{0x01}; // 2-state switches (1 byte payload)
        inline static constexpr std::byte Prop{0x02};
        inline static constexpr std::byte RequestTelemetry{0x03};
        inline static constexpr std::byte RequestTelemetryItem{0x04};
        inline static constexpr std::byte RequestConfigItem{0x05};
        inline static constexpr std::byte RequestDeviceInfo{0x06};
        inline static constexpr std::byte Set4{0x07}; // 4-state switches (2-bytes payload)
    }
    namespace CcCommand {
        inline static constexpr std::byte SetAltData{0x01}; // Index: [0, 255], value 8bit
        inline static constexpr std::byte SetAltChunk{0x02};
        inline static constexpr std::byte SetChannel{0x03}; // Index: [0, 63], value 16bit
    }
    namespace PacketIndex {
        // inline static constexpr uint8_t address = 0;
        inline static constexpr uint8_t sync = 0;
        inline static constexpr uint8_t length = 1;
        inline static constexpr uint8_t type = 2;
        inline static constexpr uint8_t dest = 3;
        inline static constexpr uint8_t src = 4;
    }
    namespace ArduPilotTunnel {
        namespace Schottel {
            inline static constexpr uint16_t AppId = 6000;
            namespace Type {
                inline static constexpr uint8_t CombinedTelemetry = 0x00;
                inline static constexpr uint8_t DeviceInfo = 0x01;
            }
        }
        namespace Switch {
            inline static constexpr uint16_t AppId = 6010;
            namespace Type {
                inline static constexpr uint8_t ConfigItem = 0x00;
                inline static constexpr uint8_t DeviceInfo = 0x01;
            }
        }
    }
    static inline constexpr uint8_t  maxMessageSize = 64;
    static inline constexpr uint8_t  minMessageSize = 4;
    static inline constexpr uint8_t  maxPayloadSize = 60;
    static inline constexpr uint8_t  maxExtendedPayloadSize = 58;

    static inline constexpr uint8_t  ValueBits = 11;
    static inline constexpr uint16_t ValueMask = ((1 << ValueBits) - 1);

    static inline constexpr uint16_t CenterValue = 0x3e0; // same center and range as sbus

    inline static constexpr int min = 172;
    inline static constexpr int max = 1811;
    inline static constexpr int span = (max - min) / 2;
    inline static constexpr int amp = (max - min);
    inline static constexpr int mid = (max + min) / 2;

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
    // template<typename C>
    // requires(std::is_pointer_v<C>)
    // static inline void pack(const std::array<uint16_t, 16>& channels, C out) {
    //     Channels packed;
    //     packed.ch0 = channels[0];
    //     packed.ch1 = channels[1];
    //     packed.ch2 = channels[2];
    //     packed.ch3 = channels[3];
    //     packed.ch4 = channels[4];
    //     packed.ch5 = channels[5];
    //     packed.ch6 = channels[6];
    //     packed.ch7 = channels[7];
    //     packed.ch8 = channels[8];
    //     packed.ch9 = channels[9];
    //     packed.ch10 = channels[10];
    //     packed.ch11 = channels[11];
    //     packed.ch12 = channels[12];
    //     packed.ch13 = channels[13];
    //     packed.ch14 = channels[14];
    //     packed.ch15 = channels[15];

    //     const uint8_t* const ptr = (uint8_t*)&packed;

    //     *out++ = 0xc8;
    //     *out++ = (sizeof(Channels) + 2);
    //     *out++ = (uint8_t)Type::Channels;
    //     CRC8 crc;
    //     crc += Type::Channels;
    //     static_assert(sizeof(Channels) == 22);
    //     for(uint8_t i = 0; i < sizeof(Channels); ++i) {
    //         crc += ptr[i];
    //         *out++ = ptr[i];
    //     }
    //     *out++ = crc;
    // }
    // template<typename T, auto L>
    // static inline uint8_t pack(const std::array<uint16_t, 16>& channels, std::array<T, L>& out) {
    //     static_assert(L > 26);
    //     Channels packed;
    //     packed.ch0 = channels[0];
    //     packed.ch1 = channels[1];
    //     packed.ch2 = channels[2];
    //     packed.ch3 = channels[3];
    //     packed.ch4 = channels[4];
    //     packed.ch5 = channels[5];
    //     packed.ch6 = channels[6];
    //     packed.ch7 = channels[7];
    //     packed.ch8 = channels[8];
    //     packed.ch9 = channels[9];
    //     packed.ch10 = channels[10];
    //     packed.ch11 = channels[11];
    //     packed.ch12 = channels[12];
    //     packed.ch13 = channels[13];
    //     packed.ch14 = channels[14];
    //     packed.ch15 = channels[15];

    //     const uint8_t* const ptr = (uint8_t*)&packed;

    //     uint8_t n = 0;
    //     out[n++] = 0xc8;
    //     out[n++] = (sizeof(Channels) + 2);
    //     out[n++] = (uint8_t)Type::Channels;
    //     CRC8 crc;
    //     crc += Type::Channels;
    //     static_assert(sizeof(Channels) == 22);
    //     for(uint8_t i = 0; i < sizeof(Channels); ++i) {
    //         crc += ptr[i];
    //         out[n++] = ptr[i];
    //     }
    //     out[n++] = crc;
    //     return n;
    // }
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
        using value_type = T;
        enum Type {U8 = 0, I8, U16, I16, F32 = 8, Sel, Str, Folder, Info, Command, OutOfRange = 127, Hidden = 0b1000'0000};
        uint8_t parent{};
        Type type = Type::U8;
        inline void hide(const bool h) {
            if (h) {
                type = Type(type | Type::Hidden);
            }
            else {
                type = Type(type & ~Type::Hidden);
            }
        }
        const char* name = nullptr;
        const char* options = nullptr;
        T* value_ptr = nullptr;
        inline T value() const {
            if (value_ptr) {
                return *value_ptr;
            }
            else {
                return val;
            }
        }
        inline void value(const T v) {
            if (value_ptr) {
                *value_ptr = v;
            }
            else {
                val = v;
            }
        }
        T min{0};
        T max{0};
        bool (*cb)(T){nullptr};
        T def{min};
        uint8_t units{0}; // should be a c-string??? (not allways: for STRING this is maxLength of the string when written)
        T val{def};
        char* stringValue = nullptr;
        const char* unitString = nullptr;
        uint8_t prec = 1;
        uint32_t fstep = 1;
        inline uint16_t size() const {
            uint16_t l = 4 * sizeof(T) + 1 + 1 + 1; // 4(value, default, min, max) + type + parent + units
            if (name) {
                l += strlen(name) + 1;
            }
            if (options) {
                l += strlen(options) + 1;
            }
            if (stringValue) {
                l += strlen(stringValue) + 1;
            }
            if (unitString) {
                l += strlen(unitString) + 1;
            }
            return l;
        }
        // template<typename U, typename C>
        // inline void serializeNumerical(C& c) {
        //     etl::serializeBE<U>(value(), c);
        //     etl::serializeBE<U>(min, c);
        //     etl::serializeBE<U>(max, c);
        //     etl::serializeBE<U>(def, c);
        //     etl::push_back_ntbs_or_emptyString(unitString, c);
        // }
        // template<typename C, typename PC>
        // inline void serialize(C& c, const PC& params, const Lua::CmdStep step = Lua::CmdStep::Idle, const uint8_t index = 0xff) {
        //     c.push_back((uint8_t)parent);
        //     c.push_back((uint8_t)type);
        //     etl::push_back_ntbs_or_emptyString(name, c);
        //     const uint8_t typ = type & 0b0111'1111;
        //     if (typ <= Type::I8) {
        //         serializeNumerical<uint8_t>(c);
        //     }
        //     else if (typ <= Type::I16) {
        //         serializeNumerical<uint16_t>(c);
        //     }
        //     else if (typ <= Type::F32) {
        //         serializeNumerical<uint32_t>(c);
        //         c.push_back(prec);
        //         etl::serializeBE(fstep, c);
        //     }
        //     else if (typ == Type::Sel) {
        //         etl::push_back_ntbs_or_emptyString(options, c);
        //         c.push_back((uint8_t)value());
        //         c.push_back((uint8_t)min);
        //         c.push_back((uint8_t)max);
        //         c.push_back((uint8_t)def);
        //         etl::push_back_ntbs_or_emptyString(unitString, c);
        //     }
        //     else if (typ == Type::Str) {
        //         etl::push_back_ntbs_or_emptyString(stringValue, c);
        //         c.push_back(units); // max.length
        //     }
        //     else if (typ == Type::Folder) {
        //         std::array<uint8_t, 32> children;
        //         uint8_t k = 0;
        //         for(uint8_t i = 0; (i < params.size()) && (k < children.size()); ++i) {
        //             if (params[i].parent == index) {
        //                 if (index == 0) {
        //                     if (i != 0) { // don't list root folder itself
        //                         children[k++] = i;
        //                     }
        //                 }
        //                 else {
        //                     children[k++] = i;
        //                 }
        //             }
        //         }
        //         for(uint8_t i = 0; i < k; ++i) {
        //             c.push_back(children[i]);
        //         }
        //         c.push_back((uint8_t)0xff);
        //         // etl::push_back_ntbs_or_emptyString(options, c);
        //     }
        //     else if (typ == Type::Info) {
        //         etl::push_back_ntbs_or_emptyString(options, c);
        //     }
        //     else if (typ == Type::Command) {
        //         switch(step) {
        //         case Lua::CmdStep::Idle:
        //             c.push_back((uint8_t)Lua::CmdStep::Idle);
        //             c.push_back((uint8_t)def); // timeout
        //             etl::push_back_ntbs_or_emptyString(options, c);
        //             break;
        //         case Lua::CmdStep::Click:
        //             if (cb && cb((value_type)Lua::CmdStep::Click)) {
        //                 c.push_back((uint8_t)Lua::CmdStep::AskConfirm);
        //             }
        //             else {
        //                 c.push_back((uint8_t)Lua::CmdStep::Executing);
        //             }
        //             c.push_back((uint8_t)def); // timeout
        //             etl::push_back_ntbs_or_emptyString(options, c);
        //             break;
        //         case Lua::CmdStep::Executing:
        //             if (cb) {
        //                 cb((value_type)Lua::CmdStep::Confirmed);
        //             }
        //             c.push_back((uint8_t)Lua::CmdStep::Executing);
        //             c.push_back((uint8_t)def); // timeout
        //             etl::push_back_ntbs_or_emptyString(options, c);
        //             break;
        //         case Lua::CmdStep::AskConfirm:
        //             break;
        //         case Lua::CmdStep::Confirmed:
        //             break;
        //         case Lua::CmdStep::Cancel:
        //             if (cb) {
        //                 cb((value_type)Lua::CmdStep::Cancel);
        //             }
        //             break;
        //         case Lua::CmdStep::Query:
        //             if (cb && cb((value_type)Lua::CmdStep::Query)) {
        //                 c.push_back((uint8_t)Lua::CmdStep::Executing);
        //             }
        //             else {
        //                 c.push_back((uint8_t)Lua::CmdStep::Idle);
        //             }
        //             c.push_back((uint8_t)def); // timeout
        //             etl::push_back_ntbs_or_emptyString(options, c);
        //             break;
        //         }
        //     }
        // }
    };

    template<typename Config>
    struct Adapter {
        using dbg = Config::debug;
        using cb =  Config::cb;
        enum class State : uint8_t {Undefined,
                                    GotAddress, GotLength,
                                    // Link, Channels, Ping, Info, ParameterEntry, ParameterRead, ParameterWrite,
                                    Data, Command};

        static inline bool process(const std::byte b) {
            ++mBytesCounter;
            switch(mState) {
            case State::Undefined:
                csum.reset();
                if ((b == Crsf::Address::StartByte) ||
                    (b == Crsf::Address::TX) ||
                    (b == Crsf::Address::Broadcast))
                {
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
                if (b == Crsf::Type::Command) {
                    mState = State::Command;
                }
                else {
                    mState = State::Data;
                }
                break;
            case State::Command:
                if (mPayloadIndex >= mLength) {
                    if (csum == b) {
                        ++mCommandPackagesCounter;
                        ++mPackagesCounter;
                        if (mData[0] == Crsf::Address::Controller) {
                            if (mData[2] == Crsf::CommandType::Switch) {
                                const std::byte command = mData[3];
                                if (command == Crsf::SwitchCommand::Set) {
                                    if (mData[4] == mModuleAddress) {
                                        etl::outl<dbg>("set: "_pgm, mData[5]);
                                        cb::set(mData[5]);
                                    }
                                }
                                else if (command == Crsf::SwitchCommand::Set4) {
                                    if (mData[4] == mModuleAddress) {
                                        const uint16_t sw = (((uint16_t)mData[5]) << 8) + (uint8_t)mData[6];
                                        std::byte sw8{};
                                        for(uint8_t i = 0; i < 8; ++i) {
                                            const uint8_t s = (sw >> (2 * i)) & 0b11;
                                            if (s > 0) {
                                                sw8 |= std::byte(1 << i);
                                            }
                                        }
                                        etl::outl<dbg>("set4: "_pgm, sw8);
                                        cb::set(sw8);
                                    }
                                }
                            }
                        }
                    }
                    mState = State::Undefined;
                }
                else {
                    csum += b;
                    mData[mPayloadIndex] = b;
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
            return true;
        }
        static inline void ratePeriodic() {}

        static inline uint16_t commandPackages() {
            return mCommandPackagesCounter;
        }

        private:
        inline static CRC8 csum;
        inline static State mState = State::Undefined;
        inline static std::array<std::byte, maxMessageSize> mData;
        inline static std::byte mModuleAddress{};
        inline static uint8_t mPayloadIndex{};
        inline static uint8_t mLength{};
        inline static uint16_t mPackagesCounter{};
        inline static uint16_t mCommandPackagesCounter{};
        inline static uint16_t mDataPackagesCounter{};
        inline static uint16_t mBytesCounter{};
    };
}

