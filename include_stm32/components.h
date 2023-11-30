#pragma once

#include "mcu/mcu.h"

#include <type_traits>

namespace Mcu::Components {
    template<typename Port, uint8_t N> 
    struct Pin {
        using port_t = Port;
        using number_t = std::integral_constant<uint8_t, N>;
    };
    template<uint8_t N> 
    struct Timer {
        using number_t = std::integral_constant<uint8_t, N>;
    };
}
