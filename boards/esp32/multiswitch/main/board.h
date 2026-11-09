#pragma once

#include <array>
#include <cstdint>

namespace Board {
    // GPIO2/8/9 are strapping pins, GPIO12..17 belong to flash, 18/19 to USB.
    inline constexpr std::array<int, 8> outputs{0, 1, 3, 4, 5, 6, 7, 10};
    inline constexpr int crsfRx = 20;
    inline constexpr int crsfTx = 21;
    inline constexpr int led = 8;
    inline constexpr bool ledActiveLow = true;
    inline constexpr int button = 9;
    inline constexpr uint32_t hardwareVersion = 13;
    inline constexpr uint32_t softwareVersion = 42;
}
