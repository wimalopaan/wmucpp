#pragma once

#include "crsf.h"

template<typename Leds>
struct CrsfCommandCallback {
    static inline void set(const std::byte data) {
        std::byte mask = 0b1_B;
        for(uint8_t i = 0; i < 8; ++i) {
            if (std::any(data & mask)) {
                Leds::set(i, true);
            }
            else {
                Leds::set(i, false);
            }
            mask <<= 1;
        }
    }
};
