#pragma once

#include <cstdint>
#include <std/utility>

namespace AVR {
    namespace Series0 {
        struct GPIOR {
            volatile std::byte data;

            template<int N> struct Address;
        };
    }
}
