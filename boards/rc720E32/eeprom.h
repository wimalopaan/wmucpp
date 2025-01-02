#pragma once

#include <cstdint>
#include <array>

struct EEProm {
    using eeprom_value_t = uint16_t;

    struct Pair {
        eeprom_value_t first;
        eeprom_value_t second;
    };

    // Generell
    eeprom_value_t mode = 0;
    // CRSF
#ifdef CRSF_ADDRESS
    eeprom_value_t address = CRSF_ADDRESS;
#else
    eeprom_value_t address = 0xc8;
#endif

    eeprom_value_t input_stream = 0; // CRSF, Pulse

    // Input channels
    std::array<Pair, 2> channels{{{0, 1}, {2, 3}}};

    // escs
    std::array<eeprom_value_t, 2> deadbands{10, 10};

    // servos
    eeprom_value_t offset1  = 0; // 0...360
    eeprom_value_t speed1 = 100;
    eeprom_value_t offset2  = 0;
    eeprom_value_t speed2 = 100;

    // (s)bus out
    // phi1/2, amp1/2 are injected in these channels
    eeprom_value_t inject  = 1;
    eeprom_value_t amp1_ch = 0;
    eeprom_value_t phi1_ch = 1;
    eeprom_value_t amp2_ch = 2;
    eeprom_value_t phi2_ch = 3;

#ifdef TEST_EEPROM
    eeprom_value_t crsf_hd_mode = 1;
#else
    eeprom_value_t crsf_hd_mode = 0;
#endif
    eeprom_value_t crsf_fd_aux_mode = 0;
    std::array<eeprom_value_t, 2> out_mode_srv{2, 2};
#ifdef TEST_EEPROM
    // std::array<eeprom_value_t, 2> out_mode_esc{2, 0}; // Esc32 Ascii
    std::array<eeprom_value_t, 2> out_mode_esc{3, 0}; // VEsc
#else
    std::array<eeprom_value_t, 2> out_mode_esc{0, 0};
#endif
    std::array<eeprom_value_t, 2> tlm_mode_esc{0, 0};

    std::array<eeprom_value_t, 2> esc_mid{50, 50}; // [0...200]

    eeprom_value_t prerun_check = 1;
};
