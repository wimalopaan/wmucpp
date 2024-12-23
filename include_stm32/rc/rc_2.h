#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <numbers>

#include "byte.h"
#include "meta.h"

namespace RC {
    namespace Protokoll {
        namespace ESCape {
            static inline constexpr uint32_t baudrate{460'800};
            static inline constexpr int min = -2000;
            static inline constexpr int mid = 0;
            static inline constexpr int max = 2000;
            static inline constexpr int span = (max - min) / 2;
            static inline constexpr int amp = (max - min);
            template<typename T = std::byte>
            struct CheckSum {
                T operator+=(const T b) {
                    mValue = T(tbl[uint8_t(mValue ^ b)]);
                    return b;
                }
                void reset() {
                    mValue = T{0};
                }
                operator T() const {
                    return mValue;
                }
            private:
                T mValue{};
                inline static constexpr uint8_t tbl[] = {
                    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
                    0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
                    0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
                    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
                    0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
                    0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
                    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
                    0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
                    0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
                    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
                    0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
                    0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
                    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
                    0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
                    0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
                    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3,
                };
            };
            namespace Ascii {
                static constexpr uint32_t baudrate{38'400};
            }
        }
        namespace Crsf {
            namespace V4 {
                static constexpr uint32_t baudrate{420'000};
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
                    inline static constexpr std::byte Set{0x01};
                    inline static constexpr std::byte Prop{0x02};
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
                static inline constexpr uint8_t  minMessageSize = 4;
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
            }
        }
        namespace IBus {
            namespace V2 {
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
                static constexpr uint32_t baudrate{115'200};
                static constexpr uint8_t numberOfChannels{18};
                inline static constexpr uint16_t min = 988;
                inline static constexpr uint16_t max = 2011;
                inline static constexpr uint16_t amp = (max - min);
                inline static constexpr uint16_t span = (max - min) / 2;
                inline static constexpr uint16_t mid = (max + min) / 2;
                using value_type = etl::ranged<min, max>;
                using index_type = etl::ranged<0, numberOfChannels - 1>;
                struct CheckSum final {
                    inline uint8_t operator+=(const uint8_t b) {
                        mSum -= b;
                        return b;
                    }
                    inline uint8_t highByte() const {
                        return mSum >> 8;
                    }
                    inline uint8_t lowByte() const {
                        return mSum;
                    }
                    inline void highByte(const uint8_t hb) {
                        mH = hb;
                    }
                    inline void lowByte(const uint8_t lb){
                        mL = lb;
                    }
                    inline explicit operator bool() const {
                        return (lowByte() == mL) && (highByte() == mH);
                    }
                    private:
                    uint8_t mH{};
                    uint8_t mL{};
                    uint16_t mSum = std::numeric_limits<uint16_t>::max();
                };
            }
        }
        namespace SBus {
            namespace V2 {
                using namespace etl::literals;
                static constexpr uint32_t baudrate{100'000};
                static constexpr uint8_t numberOfChannels{16};
                static inline constexpr std::byte start_byte = 0x0f_B;
                static inline constexpr std::byte end_byte = 0x00_B;

                static inline constexpr std::byte ch17 = 0x01_B;
                static inline constexpr std::byte ch18 = 0x02_B;
                static inline constexpr std::byte frameLost = 0x04_B;
                static inline constexpr std::byte failSafe = 0x08_B;

                inline static constexpr int min = 172;
                inline static constexpr int max = 1811;
                inline static constexpr int span = (max - min) / 2;
                inline static constexpr int amp = (max - min);
                inline static constexpr int mid = (max + min) / 2;

                using value_type = etl::ranged<min, max>;
                using index_type = etl::ranged<0, 15>;
            }
        }
        namespace Hott {
            namespace SumDV3 {
                namespace V2 {
                    static constexpr uint32_t baudrate{115'200};
                    static constexpr uint8_t numberOfChannels = 32;

                    static constexpr uint8_t start_code = 0xa8;
                    static constexpr uint8_t version_code1 = 0x01;
                    static constexpr uint8_t version_code3 = 0x03;
                    static constexpr uint16_t ExtendedLow = 0x1c20; // 7200
                    static constexpr uint16_t MinValue = 0x2260; // 8800
                    static constexpr uint16_t CenterValue = 0x2ee0; // 12000
                    static constexpr uint16_t MaxValue = 0x3b60; // 15200
                    static constexpr uint16_t ExtendedHigh = 0x41a0; // 16800

                    struct Crc16 {
                        inline void reset() {
                            sum = 0;
                        }
                        inline uint8_t operator+=(const uint8_t v) {
                            sum = sum ^ (((uint16_t)v) << 8);
                            for(uint8_t i = 0; i < 8; ++i) {
                                if (sum & 0x8000) {
                                    sum = (sum << 1) ^ crc_polynome;
                                }
                                else {
                                    sum = (sum << 1);
                                }
                            }
                            return v;
                        }
                        inline operator uint16_t() const {
                            return sum;
                        }
                    private:
                        static constexpr uint16_t crc_polynome = 0x1021;
                        uint16_t sum{};
                    };
                }
            }
        }
        namespace SPort {
            // https://github.com/jcheger/frsky-arduino/tree/master/FrskySP
            // https://github.com/opentx/opentx/blob/2.3/radio/src/telemetry/frsky.h
            using namespace std::literals::chrono_literals;
            using namespace Units::literals;
            using namespace etl::literals;

            namespace V2 {
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
            }
        }
    }
}
