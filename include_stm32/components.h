#pragma once

#include "mcu/mcu.h"

#include <type_traits>

namespace Mcu::Components {
    template<typename L>
    struct Port {
        using letter_t = L;
    };
    template<typename Port, uint8_t N> 
    struct Pin {
        using port_t = Port;
        using number_t = std::integral_constant<uint8_t, N>;
    };
    template<uint8_t N>
    struct Timer {
        using number_t = std::integral_constant<uint8_t, N>;
    };
    template<uint8_t N, typename MCU = DefaultMcu>
    struct Adc {
        using number_t = std::integral_constant<uint8_t, N>;
    };
    template<uint8_t N>
    struct Usart {
        using number_t = std::integral_constant<uint8_t, N>;
    };
    struct Rcc {
    };
    template<uint8_t N>
    struct Dma {
        using number_t = std::integral_constant<uint8_t, N>;
    };
    template<uint8_t N>
    struct DmaChannel {
        using number_t = std::integral_constant<uint8_t, N>;
    };
    template<uint8_t N>
    struct Comparator {
        using number_t = std::integral_constant<uint8_t, N>;
    };
}
