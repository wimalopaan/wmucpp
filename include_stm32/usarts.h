#pragma once

namespace Mcu::Stm {
    namespace Uarts {
        enum class Mode : uint8_t {TxOnly, RxOnly, HalfDuplex, FullDuplex};

        static constexpr uint16_t LPUART_PRESCALER_TAB[] = {1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256};

        template<uint8_t N> struct Properties;
#ifdef STM32G4
        template<> struct Properties<1> {
            static inline constexpr uint8_t dmamux_rx_src = 24;
            static inline constexpr uint8_t dmamux_tx_src = 25;
        };
        template<> struct Properties<2> {
            static inline constexpr uint8_t dmamux_rx_src = 26;
            static inline constexpr uint8_t dmamux_tx_src = 27;
        };
        template<> struct Properties<3> {
            static inline constexpr uint8_t dmamux_rx_src = 28;
            static inline constexpr uint8_t dmamux_tx_src = 29;
        };
        template<> struct Properties<4> {
            static inline constexpr uint8_t dmamux_rx_src = 30;
            static inline constexpr uint8_t dmamux_tx_src = 31;
        };
        template<> struct Properties<5> {
            static inline constexpr uint8_t dmamux_rx_src = 32;
            static inline constexpr uint8_t dmamux_tx_src = 33;
        };
        template<> struct Properties<101> {
            static inline constexpr uint8_t dmamux_rx_src = 34;
            static inline constexpr uint8_t dmamux_tx_src = 35;
        };
#endif
#ifdef STM32G0
        template<> struct Properties<1> {
            static inline constexpr uint8_t dmamux_rx_src = 50;
            static inline constexpr uint8_t dmamux_tx_src = 51;
        };
        template<> struct Properties<2> {
            static inline constexpr uint8_t dmamux_rx_src = 52;
            static inline constexpr uint8_t dmamux_tx_src = 53;
        };
        template<> struct Properties<3> {
            static inline constexpr uint8_t dmamux_rx_src = 54;
            static inline constexpr uint8_t dmamux_tx_src = 55;
        };
        template<> struct Properties<4> {
            static inline constexpr uint8_t dmamux_rx_src = 56;
            static inline constexpr uint8_t dmamux_tx_src = 57;
        };
        template<> struct Properties<5> {
            static inline constexpr uint8_t dmamux_rx_src = 74;
            static inline constexpr uint8_t dmamux_tx_src = 75;
        };
        template<> struct Properties<6> {
            static inline constexpr uint8_t dmamux_rx_src = 76;
            static inline constexpr uint8_t dmamux_tx_src = 77;
        };
        template<> struct Properties<101> {
            static inline constexpr uint8_t dmamux_rx_src = 14;
            static inline constexpr uint8_t dmamux_tx_src = 15;
        };
        template<> struct Properties<102> {
            static inline constexpr uint8_t dmamux_rx_src = 64;
            static inline constexpr uint8_t dmamux_tx_src = 65;
        };
#endif
    }

    template<>
    struct Address<Mcu::Components::Usart<1>> {
        static inline constexpr uintptr_t value = USART1_BASE;
    };
    template<>
    struct Address<Mcu::Components::Usart<2>> {
        static inline constexpr uintptr_t value = USART2_BASE;
    };
#ifdef USART3_BASE
    template<>
    struct Address<Mcu::Components::Usart<3>> {
        static inline constexpr uintptr_t value = USART3_BASE;
    };
#endif
#ifdef USART4_BASE
    template<>
    struct Address<Mcu::Components::Usart<4>> {
        static inline constexpr uintptr_t value = USART4_BASE;
    };
#endif
#ifdef UART4_BASE
    template<>
    struct Address<Mcu::Components::Usart<4>> {
        static inline constexpr uintptr_t value = UART4_BASE;
    };
#endif
#ifdef USART5_BASE
    template<>
    struct Address<Mcu::Components::Usart<5>> {
        static inline constexpr uintptr_t value = USART5_BASE;
    };
#endif
#ifdef UART5_BASE
    template<>
    struct Address<Mcu::Components::Usart<5>> {
        static inline constexpr uintptr_t value = UART5_BASE;
    };
#endif
#ifdef USART6_BASE
    template<>
    struct Address<Mcu::Components::Usart<6>> {
        static inline constexpr uintptr_t value = USART6_BASE;
    };
#endif
#ifdef LPUART1_BASE
    template<>
    struct Address<Mcu::Components::Usart<101>> {
        static inline constexpr uintptr_t value = LPUART1_BASE;
    };
#endif
#ifdef LPUART2_BASE
    template<>
    struct Address<Mcu::Components::Usart<102>> {
        static inline constexpr uintptr_t value = LPUART2_BASE;
    };
#endif
}
