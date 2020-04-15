#pragma once

#include <cstdint>
#include <std/utility>

namespace AVR {
    namespace Series0 {
        struct GPIOR {
            volatile uint8_t data;

            template<int N> struct Address;
        };
    }
}
