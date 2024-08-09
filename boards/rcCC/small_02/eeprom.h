#pragma once

#include <cstdint>
#include <array>

struct EEProm {
    struct Output {
        uint8_t mode = 0;
        uint8_t channel = 0;
        uint8_t sw = 0;
    };
    struct Switch {
        uint8_t mode = 0;
        uint8_t pwmDuty = 1;
    };

    uint8_t address = 0;

    uint8_t serial1_mode = 0;
    uint8_t serial2_mode = 0;
    uint8_t aux1_mode = 0;
    uint8_t aux2_mode = 0;
    uint8_t telemetry_polepairs = 1;

    std::array<Output, 4> outputs = {{
                                         {
                                             .mode = 0, .channel = 0, .sw = 0
                                         },
                                         {
                                             .mode = 0, .channel = 1, .sw = 1
                                         },
                                         {
                                             .mode = 0, .channel = 2, .sw = 2
                                         },
                                         {
                                             .mode = 0, .channel = 3, .sw = 3
                                         }
                                     }};

    uint8_t tlc_1_address = 0;
    uint8_t tlc_1_groupmode = 0;
    std::array<Switch, 8> tlc_1{};

    uint8_t dummy = 42;
};

