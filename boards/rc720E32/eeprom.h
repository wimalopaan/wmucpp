#pragma once

#include <cstdint>
#include <array>

struct Pair {
    uint16_t first;
    uint16_t second;
};

struct EEProm {
    // CRSF
    uint16_t address = 192;

    uint16_t input_stream = 0; // CRSF, Pulse

    // Input channels
    std::array<Pair, 2> channels{{{0, 1}, {2, 3}}};

    // escs
    std::array<uint16_t, 2> deadbands{10, 10};

    // servos
    uint16_t offset1  = 0; // 0...360
    uint16_t speed1 = 100;
    uint16_t offset2  = 0;
    uint16_t speed2 = 100;

    // (s)bus out
    // phi1/2, amp1/2 are injected in these channels
    uint16_t inject  = 1;
    uint16_t amp1_ch = 0;
    uint16_t phi1_ch = 1;
    uint16_t amp2_ch = 2;
    uint16_t phi2_ch = 3;

    uint16_t crsf_hd_mode = 6;
    uint16_t crsf_fd_aux_mode = 0;

    std::array<uint16_t, 2> out_mode_srv{2, 2};
    std::array<uint16_t, 2> out_mode_esc{0, 0};
    std::array<uint16_t, 2> tlm_mode_esc{0, 0};

    uint16_t prerun_check = 1;
};
