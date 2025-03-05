#pragma once

#include "usart_2.h"
#include "mcu/alternate.h"

template<uint8_t N, typename Config, typename MCU = DefaultMcu>
struct SerialBuffered {
    using pin = Config::pin;
    static inline void init() {
        static uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;
        uart::init();
        pin::afunction(af);
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
    private:
    struct UartConfig {
        using Clock = Config::clock;
        using ValueType = unsigned char;
        static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::TxOnly;
        static inline constexpr uint32_t baudrate = 115'200;
        static inline constexpr bool rxtxswap = Config::rxtxswap;
        static inline constexpr bool fifo = []{
            if (N <= 3) return true;
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
