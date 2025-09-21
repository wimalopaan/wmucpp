/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
    template<uint8_t N>
    struct I2C {
        using number_t = std::integral_constant<uint8_t, N>;
    };
    template<uint8_t N>
    struct SPI {
        using number_t = std::integral_constant<uint8_t, N>;
    };
    template<uint8_t N>
    struct Dac {
        using number_t = std::integral_constant<uint8_t, N>;
    };
    struct Rcc {
    };
    template<uint8_t N>
    struct Dma {
        using number_t = std::integral_constant<uint8_t, N>;
    };
    template<typename Controller, uint8_t N>
    struct DmaChannel {
        using controller = Controller;
        using number_t = std::integral_constant<uint8_t, N>;
    };
    template<uint8_t N>
    struct DmaRequestGenerator{
        using number_t = std::integral_constant<uint8_t, N>;
    };
    template<uint8_t N>
    struct Comparator {
        using number_t = std::integral_constant<uint8_t, N>;
    };
}
