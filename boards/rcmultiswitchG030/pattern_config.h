#pragma once

#include <array>
#include <cstdint>

namespace External::Pattern {
    struct EEProm {
        uint8_t type = 0;
        std::array<uint8_t, 8> member{1, 2, 3, 4, 5, 6, 7, 8};
        uint8_t onTime = 33; // *10 ms
        uint8_t offTime = 1;
        uint8_t next_address = 3;
        uint8_t group = 1; // 0: no group
    };
}
