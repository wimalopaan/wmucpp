/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include <array>
#include <utility>

#include <etl/stringbuffer.h>

#include <external/units/physical.h>

#pragma pack(push)
#pragma pack(1)

namespace Hott {
    namespace Units {
        using battery_voltage_t = External::Units::voltage<uint16_t, std::ratio<1,10>>;
        using cell_voltage_t = External::Units::voltage<uint8_t, std::ratio<1,500>>;
        using current_t = External::Units::ampere<uint16_t, std::ratio<1,10>>;

        template<typename ADConv, typename Voltage, typename Scale = std::ratio<1,1>>
        struct Converter {
            using adc_type = ADConv;
            using raw_type = ADConv::value_type;
            using voltage_type = Voltage;
            using representation_type = typename Voltage::value_type;
            using scale_type = typename Voltage::divider_type;
            
            static inline constexpr uint8_t rep_bits = (etl::numberOfBits<representation_type>() == 8) ? 16 : etl::numberOfBits<representation_type>();
//            using std::integral_constant<uint8_t, rep_bits>::_;
            
            static inline constexpr uint8_t reso_bits = ADConv::reso_type::bits;
            static inline constexpr uint8_t nominator_bits = rep_bits - reso_bits;
            static_assert(nominator_bits <= 8);
//            using std::integral_constant<uint8_t, nominator_bits>::_;
            
            static inline constexpr uint16_t nominator_value = (1 << nominator_bits) - 1;
//            using std::integral_constant<uint8_t, nominator_value>::_;
            
            static inline constexpr uint16_t denominator_value = (((float)nominator_value * scale_type::nom) / scale_type::denom) / ((adc_type::VBit * Scale::nom) / Scale::denom);
//            using std::integral_constant<uint16_t, denominator_value>::_;
            
            static inline constexpr voltage_type convert(auto raw) {
                auto v = (raw * nominator_value) / denominator_value;
                return {(representation_type)v};
            }
            static inline constexpr voltage_type convert(const raw_type& raw) {
                auto v = (raw.toInt() * nominator_value) / denominator_value;
                return {(representation_type)v};
            }
        };
    }
    
    using namespace std::literals::chrono;
    
    enum class key_t : uint8_t {right = 14, left = 7, up = 11, down = 13, set = 9, nokey = 0x0f};
    
    static inline constexpr auto hottDelayBeforeAnswer = 5000_us;
    static inline constexpr auto hottDelayBetweenBytes = 2000_us;
    
    static inline constexpr std::byte ascii_start_code = 0x7b_B;
    static inline constexpr std::byte start_code = 0x7c_B;
    static inline constexpr std::byte end_code = 0x7d_B;
    static inline constexpr std::byte ascii_msg_start = 0x7f_B;
    static inline constexpr std::byte msg_start = 0x80_B;
    static inline constexpr std::byte broadcast_code = 0x80_B;
    
    static inline constexpr std::byte rec_code = 0x00_B;
    static inline constexpr std::byte var_code = 0x09_B;
    static inline constexpr std::byte gps_code = 0x0a_B;
    static inline constexpr std::byte esc_code = 0x0c_B;
    static inline constexpr std::byte gam_code = 0x0d_B;
    static inline constexpr std::byte air_code = 0x0e_B;

    static inline constexpr bool valid_code(const std::byte code) {
        return (code >= var_code) && (code <= air_code);
    }
    
    static inline constexpr std::byte binary_id(const std::byte code) {
        return 0x80_B | (code & 0x0f_B);
    }
    static inline constexpr std::byte ascii_id(const std::byte code) {
        return ((code & 0x0f_B) << 4);
//        return 0x0f_B | ((code & 0x0f_B) << 4);
    }    
    
    struct TextMsg {
        inline static constexpr uint8_t rows = 8;
        inline static constexpr uint8_t columns = 21;
        using value_type = uint8_t;
        using line_type = etl::StringBuffer<columns>;
        using buffer_type = std::array<line_type, rows>; 
        
        const std::byte start_byte = ascii_start_code;		//#01 Starting constant value == 0x7b
        std::byte esc{0};				//#02 Escape (higher-ranking menu in text mode or Text mode leave)
        uint8_t warning_beeps = 0;	//#03 1=A 2=B ...
        buffer_type text{};
        const std::byte stop_byte = end_code;		//#172 constant value 0x7d
        uint8_t parity = 0;			//#173 Checksum / parity
    };
    
    template<uint8_t Rows>
    requires((Rows > 0) && (Rows <= 8))
    struct VarTextMsg {
        inline static constexpr uint8_t rows = Rows;
        inline static constexpr uint8_t columns = 21;
        using value_type = uint8_t;
        using line_type = etl::StringBuffer<columns>;
        using buffer_type = std::array<line_type, rows>; 

        static inline constexpr std::byte start_byte = ascii_start_code;		//#01 Starting constant value == 0x7b
        static inline constexpr std::byte warning_beeps{0};	//#03 1=A 2=B ...
        
        std::byte esc{0};				//#02 Escape (higher-ranking menu in text mode or Text mode leave)
        buffer_type text{};
        etl::Parity<uint8_t> parity;			//#173 Checksum / parity
        
        static inline constexpr std::byte stop_byte = end_code;		//#172 constant value 0x7d
        
        static inline constexpr uint8_t text_offset = 3;  
        static inline constexpr uint8_t stop_offset = 171;  
        static inline constexpr uint8_t parity_offset = 172;  
        
        static_assert(parity_offset == (sizeof(TextMsg) - 1));
        
        inline std::byte operator[](etl::uint_ranged<uint8_t, 0, sizeof(TextMsg) - 1> index)  {
            if (index == 0) {
                parity.clear();
                parity += start_byte;
                return start_byte;
            }
            else if (index == 1) {
                parity += esc;
                return esc;
            }
            else if (index == 2) {
                parity += warning_beeps;
                return warning_beeps;    
            }
            else if (index < (sizeof(text) + text_offset)) {
                /*constexpr */const std::byte* ptr = (const std::byte*) &text;  
                const auto value = ptr[index - text_offset];
                parity += value;
                return value;
            }
            else if (index == stop_offset) {
                parity += stop_byte;
                return stop_byte;
            }
            else if (index == parity_offset) {
                return parity;
            }
            else {
                parity += std::byte{' '};
                return std::byte{' '};
            }
        }
        
    };
    
    namespace detail {
        using std_txt_msg_size = std::integral_constant<uint8_t, sizeof(TextMsg)>;
//        std_txt_msg_size::_;
        using var_txt_msg_size = std::integral_constant<uint8_t, sizeof(VarTextMsg<4>)>;
//        var_txt_msg_size::_;
    }

    struct VarioMsg {
        using value_type = uint8_t;
        const std::byte start_byte = start_code;          //#01 start byte constant value 0x7c
        const std::byte vario_sensor_id = binary_id(var_code);
        uint8_t warning_beeps = 0;
        const std::byte sensor_id = (vario_sensor_id << 4) & 0xf0_B;

        uint8_t status = 0x00;
        uint16_t height = 0x00;
        uint16_t height_max = 0x00;
        uint16_t height_min = 0x00;
        uint16_t speed = 0x00;
        uint16_t acc = 0x00;
        uint16_t acc10 = 0x00;
        
        uint8_t a[21] {};
        uint8_t afree[3] {};
        uint8_t free;
        uint8_t version;
        
        const std::byte end_byte = end_code;
        uint8_t parity = 0;
    };
    namespace detail {
        using vario_size = std::integral_constant<uint8_t, sizeof(VarioMsg)>;
//        vario_size::_;
    }

    struct GpsMsg {
        using value_type = uint8_t;
        const std::byte start_byte = start_code;          //#01 start byte constant value 0x7c
        const std::byte gps_sensor_id = binary_id(gps_code);
        uint8_t warning_beeps = 0;
        const std::byte sensor_id = (gps_sensor_id << 4) & 0xf0_B;
        uint8_t status  = 0x00;
        uint8_t status1 = 0x00;
        uint8_t dir     = 0x00;
        uint16_t speed  = 0x00;
        uint8_t d[10] {};
        uint16_t dist = 0x00;
        uint16_t height = 0x00;
        uint16_t vario  = 0x00;
        uint8_t  acc  = 0x00;
        uint8_t  sat  = 0x00;
        uint8_t  fix  = 0x00;
        uint8_t  home = 0x00;
        uint8_t angle[3] {};
        uint16_t gyro_x = 0x00;
        uint16_t gyro_y = 0x00;
        uint16_t gyro_z = 0x00;
        uint8_t  vib = 0x00;
        uint8_t  free1 = 0x00;
        uint8_t  free2 = 0x00;
        uint8_t  free3 = 0x00;
        uint8_t  ver   = 0x00;
        const std::byte end_byte = end_code;
        uint8_t parity = 0;
    };
    namespace detail {
        using gps_size = std::integral_constant<uint8_t, sizeof(GpsMsg)>;
//        gps_size::_;
    }
    
    struct AirMsg {
        using value_type = uint8_t;
        const std::byte start_byte = start_code;          //#01 start byte constant value 0x7c
        const std::byte air_sensor_id = binary_id(air_code);
        uint8_t warning_beeps = 0;
        const std::byte sensor_id = (air_sensor_id << 4) & 0xf0_B;
        uint8_t alarm1 = 0x00;
        uint8_t alarm2 = 0x00;
        uint8_t cell1l = 0x00;
        uint8_t cell2l = 0x00;
        uint8_t cell3l = 0x00;
        uint8_t cell4l = 0x00;
        uint8_t cell5l = 0x00;
        uint8_t cell6l = 0x00;
        uint8_t cell7l = 0x00;
        uint8_t cell1h = 0x00;
        uint8_t cell2h = 0x00;
        uint8_t cell3h = 0x00;
        uint8_t cell4h = 0x00;
        uint8_t cell5h = 0x00;
        uint8_t cell6h = 0x00;
        uint8_t cell7h = 0x00;
        uint16_t batt1 = 0x00;
        uint16_t batt2 = 0x00;
        uint8_t temp1  = 0x00;
        uint8_t temp2  = 0x00;
        uint16_t alt   = 0x00;
        uint16_t curr  = 0x00;
        uint16_t power = 0x00;
        uint16_t mAh   = 0x00;
        uint16_t velocity = 0x00;
        uint8_t  acc = 0x00;
        uint16_t rpm = 0x00;
        uint16_t time  = 0x00;
        uint8_t  speed = 0x00;
        uint8_t  version = 0x00;
        const std::byte end_byte = end_code;
        uint8_t parity = 0;
    };
    namespace detail {
        using air_size = std::integral_constant<uint8_t, sizeof(AirMsg)>;
//        air_size::_;
    }
    
    struct EscMsg {
        using value_type = uint8_t;
        const std::byte start_byte = start_code;        
        const std::byte esc_sensor_id = binary_id(esc_code);
        uint8_t warning_beeps = 0;
        const std::byte sensor_id = (esc_sensor_id << 4) & 0xf0_B;
        uint8_t inverse = 0x00;
        uint8_t inverse_status = 0x80;
        uint16_t voltage = 0; // #6
        uint16_t voltage_min = 0; // #8
        uint16_t capacity = 0;
        uint8_t temp = 0;
        uint8_t temp_max = 0;
        uint16_t current = 0; // #14
        uint16_t current_max = 0;
        uint16_t rpm = 0;
        uint16_t rpm_max = 0;
        uint8_t throttle = 0;
        uint16_t speed = 0;
        uint16_t speed_max = 0;
        uint8_t bec = 0;
        uint8_t bec_min = 0;
        uint8_t bec_current = 0;
        uint16_t bec_current_max = 0;
        uint8_t pwm = 0;
        uint8_t bec_temp = 0;
        uint8_t bec_temp_max = 0;
        uint8_t motor_temp = 0;
        uint8_t motor_temp_max = 0;
        uint16_t rpm2 = 0;
        uint8_t timing = 0;
        uint8_t adv_timing = 0;
        uint8_t hcurr = 0;
        uint8_t version = 0;
        const std::byte end_byte = end_code;
        uint8_t parity = 0;
    };

    struct EscMsg_1 {
        using value_type = uint8_t;
        inline static constexpr std::byte start_byte = start_code;        
        inline static constexpr std::byte esc_sensor_id = binary_id(esc_code);
        inline static constexpr std::byte end_byte = end_code;
        inline static constexpr std::byte sensor_id = (esc_sensor_id << 4) & 0xf0_B;
        inline static constexpr std::byte inverse_status = 0x80_B;

        uint16_t voltage = 0;
        uint16_t voltage_min = 0;

        uint16_t current = 0;
        uint16_t current_max = 0;

        static inline constexpr uint8_t parity_offset = sizeof(EscMsg) - 1;  
        etl::Parity<uint8_t> parity;

        
        inline std::byte operator[](etl::uint_ranged<uint8_t, 0, sizeof(EscMsg) - 1> index)  {
            if (index == 0) {
                parity.clear();
                return parity <<= start_byte;
            }
            else if (index == 1) {
                return parity <<= esc_sensor_id;
            }
            else if (index == 3) {
                return parity <<= sensor_id;
            }
            else if (index == 5) {
                return parity <<= inverse_status;
            }
            else if (index == 6) {
                return parity <<= etl::nth_byte<0>(voltage);
            }
            else if (index == 7) {
                return parity <<= etl::nth_byte<1>(voltage);
            }
            else if (index == 8) {
                return parity <<= etl::nth_byte<0>(voltage_min);
            }
            else if (index == 9) {
                return parity <<= etl::nth_byte<1>(voltage_min);
            }
            else if (index == 14) {
                return parity <<= etl::nth_byte<0>(current);
            }
            else if (index == 15) {
                return parity <<= etl::nth_byte<1>(current);
            }
            else if (index == 16) {
                return parity <<= etl::nth_byte<0>(current_max);
            }
            else if (index == 17) {
                return parity <<= etl::nth_byte<1>(current_max);
            }
            else if (index == parity_offset) {
                return parity;
            }
            else {
                return parity <<= 0x00_B;
            }
        }
    };
    
    namespace detail {
        using esc_size = std::integral_constant<uint8_t, sizeof(EscMsg)>;
//        esc_size::_;
    }
    
    struct GamMsg {
        using value_type = uint8_t;
        const std::byte start_byte = start_code;          //#01 start byte constant value 0x7c
        const std::byte gam_sensor_id = binary_id(gam_code);       //#02 EAM sensort id. constat value 0x8d=GENRAL AIR MODULE
        uint8_t warning_beeps = 0;       //#03 1=A 2=B ... 0x1a=Z  0 = no alarm
        /* VOICE OR BIP WARNINGS
                            Alarme sonore A.. Z, octet correspondant 1 à 26
                            0x00  00  0  No alarm
                            0x01  01  A
                            0x02  02  B  Negative Difference 2 B
                                0x03  03  C  Negative Difference 1 C
                            0x04  04  D
                            0x05  05  E
                            0x06  06  F  Min. Sensor 1 temp. F
                            0x07  07  G  Min. Sensor 2 temp. G
                            0x08  08  H  Max. Sensor 1 temp. H
                                    0x09  09  I  Max. Sensor 2 temp. I
                                0xA   10  J  Max. Sens. 1 voltage J
                            0xB   11  K  Max. Sens. 2 voltage K
                                        0xC   12  L
                            0xD   13  M  Positive Difference 2 M
                            0xE   14  N  Positive Difference 1 N
                            0xF   15  O  Min. Altitude O
                            0x10  16  P  Min. Power Voltage P    // We use this one for Battery Warning
                            0x11  17  Q  Min. Cell voltage Q
                            0x12  18  R  Min. Sens. 1 voltage R
                            0x13  19  S  Min. Sens. 2 voltage S
                            0x14  20  T  Minimum RPM T
                            0x15  21  U
                            0x16  22  V  Max. used capacity V
                            0x17  23  W  Max. Current W
                            0x18  24  X  Max. Power Voltage X
                            0x19  25  Y  Maximum RPM Y
                            0x1A  26  Z  Max. Altitude Z
                                */
        const std::byte sensor_id = (gam_sensor_id << 4) & 0xf0_B;             	        //#04 constant value 0xd0
        uint8_t alarm_invers1 = 0; //#05 alarm bitmask. Value is displayed inverted
        //Bit#  Alarm field
        // 0    all cell voltage
        // 1    Battery 1
        // 2    Battery 2
        // 3    Temperature 1
        // 4    Temperature 2
        // 5    Fuel
        // 6    mAh
        // 7    Altitude
        uint8_t alarm_invers2 = 0;     //#06 alarm bitmask. Value is displayed inverted
        //Bit#  Alarm Field
        // 0    main power current
        // 1    main power voltage
        // 2    Altitude
        // 3    m/s
        // 4    m/3s
        // 5    unknown
        // 6    unknown
        // 7    "ON" sign/text msg active
        inline static constexpr uint8_t numberOfCells = 6;
        std::array<uint8_t, numberOfCells> cell{};
        
        //#7 Volt Cell 1 (in 2 mV increments, 210 == 4.20 V)
        //#8 Volt Cell 2 (in 2 mV increments, 210 == 4.20 V)
        //#9 Volt Cell 3 (in 2 mV increments, 210 == 4.20 V)
        //#10 Volt Cell 4 (in 2 mV increments, 210 == 4.20 V)
        //#11 Volt Cell 5 (in 2 mV increments, 210 == 4.20 V)
        //#12 Volt Cell 6 (in 2 mV increments, 210 == 4.20 V)
        uint16_t  Battery1 = 0;                   //#13 LSB battery 1 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
        //#14 MSB
        uint16_t  Battery2 = 0;                   //#15 LSB battery 2 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
        //#16 MSB
        uint8_t temperature1 = 0;                    //#17 Temperature 1. Offset of 20. a value of 20 = 0°C
        uint8_t temperature2 = 0;                    //#18 Temperature 2. Offset of 20. a value of 20 = 0°C
        uint8_t fuel_procent = 0;                    //#19 Fuel capacity in %. Values 0--100
        //graphical display ranges: 0-100% with new firmwares of the radios MX12/MX20/...
        uint16_t fuel_ml = 0;                     //#20 LSB Fuel in ml scale. Full = 65535!
        //#21 MSB
        uint16_t rpm = 0;                         //#22 RPM in 10 RPM steps. 300 = 3000rpm
        //#23 MSB
        uint16_t altitude = 0;                  //#24 altitude in meters. offset of 500, 500 = 0m
        //#25 MSB
        uint16_t climbrate_L = 0;                 //#26 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
        //#27 MSB
        uint8_t climbrate3s = 0;                     //#28 climb rate in m/3sec. Value of 120 = 0m/3sec
        uint16_t current = 0;                     //#29 current in 0.1A steps 100 == 10,0A
        //#30 MSB current display only goes up to 99.9 A (continuous)
        uint16_t main_voltage = 0;            	//#31 LSB Main power voltage using 0.1V steps 100 == 10,0V
        //#32 MSB (Appears in GAM display right as alternate display.)
        uint16_t batt_cap = 0;                    //#33 LSB used battery capacity in 10mAh steps
        //#34 MSB
        uint16_t speed = 0;                       //#35 LSB (air?) speed in km/h(?) we are using ground speed here per default
        //#36 MSB speed
        uint8_t min_cell_volt = 0;                   //#37 minimum cell voltage in 2mV steps. 124 = 2,48V
        uint8_t min_cell_volt_num = 0;               //#38 number of the cell with the lowest voltage
        uint16_t rpm2 = 0;                        //#39 LSB 2nd RPM in 10 RPM steps. 100 == 1000rpm
        //#40 MSB
        uint8_t general_error_number = 0;      	//#41 General Error Number (Voice Error == 12)
        uint8_t pressure = 0;                        //#42 High pressure up to 16bar. 0,1bar scale. 20 == 2.0bar
        uint8_t version = 0;                         //#43 version number (Bytes 35 .43 new but not yet in the record in the display!)
        const std::byte stop_byte = end_code;                       //#44 stop byte 0x7D
        uint8_t parity = 0;                          //#45 CHECKSUM CRC/Parity (calculated dynamicaly)
    };
    namespace detail {
        using gam_size = std::integral_constant<uint8_t, sizeof(GamMsg)>;
//        gam_size::_;
    }

    template<typename Msg>
    struct code_from_type;
    
    template<>
    struct code_from_type<GamMsg> : std::integral_constant<std::byte, gam_code> {};
    template<>
    struct code_from_type<EscMsg> : std::integral_constant<std::byte, esc_code> {};
    template<>
    struct code_from_type<EscMsg_1> : std::integral_constant<std::byte, esc_code> {};
    template<>
    struct code_from_type<AirMsg> : std::integral_constant<std::byte, air_code> {};
    template<>
    struct code_from_type<VarioMsg> : std::integral_constant<std::byte, var_code> {};
    template<>
    struct code_from_type<GpsMsg> : std::integral_constant<std::byte, gps_code> {};

    template<typename M>
    struct size;
    template<>
    struct size<GamMsg> : std::integral_constant<uint8_t, sizeof(GamMsg)> {};
    template<>
    struct size<EscMsg> : std::integral_constant<uint8_t, sizeof(EscMsg)> {};
    template<>
    struct size<EscMsg_1> : std::integral_constant<uint8_t, sizeof(EscMsg)> {};
    template<>
    struct size<TextMsg> : std::integral_constant<uint8_t, sizeof(TextMsg)> {};
    template<uint8_t L>
    struct size<VarTextMsg<L>> : std::integral_constant<uint8_t, sizeof(TextMsg)> {};
    template<>
    struct size<AirMsg> : std::integral_constant<uint8_t, sizeof(AirMsg)> {};
    template<>
    struct size<VarioMsg> : std::integral_constant<uint8_t, sizeof(VarioMsg)> {};
    template<>
    struct size<GpsMsg> : std::integral_constant<uint8_t, sizeof(GpsMsg)> {};
    
}

#pragma pack(pop)
