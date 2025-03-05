#pragma once

#include <cstdint>
#include <array>

struct EEProm {
    consteval EEProm() = default;
    uint8_t mode = 0;
    uint8_t address = 0xc8;
    uint8_t controllerNumber = 1;
    uint8_t bluetooth = 0;
};
