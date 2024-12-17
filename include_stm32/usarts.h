#pragma once

static const uint16_t LPUART_PRESCALER_TAB[] =
{
  (uint16_t)1,
  (uint16_t)2,
  (uint16_t)4,
  (uint16_t)6,
  (uint16_t)8,
  (uint16_t)10,
  (uint16_t)12,
  (uint16_t)16,
  (uint16_t)32,
  (uint16_t)64,
  (uint16_t)128,
  (uint16_t)256
};

namespace Mcu::Stm {
    using namespace Units::literals;

    namespace Uarts {
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
}
