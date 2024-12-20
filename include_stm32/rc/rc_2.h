#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <numbers>

#include "byte.h"
#include "meta.h"

namespace RC {
    namespace Protokoll {
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
