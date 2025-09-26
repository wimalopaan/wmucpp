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

#include "usart_2.h"
#include "mcu/alternate.h"

template<uint8_t N, typename Config, typename MCU = DefaultMcu>
struct SerialBuffered {
    using pin = Config::pin;
    static inline void init() {
        uart::init();
        if constexpr(Mcu::Stm::V4::detail::getSwap_v<Config>) {
            static uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::RX>;
            pin::afunction(af);
        }
        else {
            static uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;
            pin::afunction(af);
        }
    }
    static inline void reset() {
        uart::reset();
        pin::analog();
    }
    static inline void put(const char c) {
        uart::put(c);
    }
    static inline void periodic() {
        uart::periodic();
    }
    static inline void ratePeriodic(){}
    private:
    struct UartConfig {
        using Clock = Config::clock;
        using ValueType = unsigned char;
        static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::TxOnly;
        static inline constexpr uint32_t baudrate = 115'200;
        static inline constexpr bool rxtxswap = Mcu::Stm::V4::detail::getSwap_v<Config>;
        static inline constexpr bool fifo = []{
#if defined(STM32G051xx) || defined(STM32G030xx) || defined(STM32G031xx)
            if (N <= 1) return true;
#else
            if (N <= 3) return true;
#endif
            if (N >= 101) return true;
            return false;
        }();
        struct Tx {
            static inline constexpr bool singleBuffer = true;
            static inline constexpr bool enable = true;
            static inline constexpr size_t size = Config::bufferSize;
        };
    };
    using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;
};
