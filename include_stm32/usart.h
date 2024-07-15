#pragma once

#include "mcu/mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu/mcu_traits.h"
#include "etl/fifo.h"

#include <type_traits>
#include <concepts>

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
    }

    template<uint8_t N, typename PA, auto Size, typename ValueType, typename Clock, typename MCU = void>
    struct Uart;

    template<uint8_t N, typename PA, auto Size, typename ValueType, typename Clock, typename MCU = void>
    using LpUart = Uart<N + 100, PA, Size, ValueType, Clock, MCU>;

    template<uint8_t N, typename PA, auto Size, typename ValueType, typename Clock, typename MCU>
        requires (
                    ((N >= 1) && (N <= 3) && std::is_same_v<Stm32G431, MCU>) ||
                    ((N >= 1) && (N <= 5) && std::is_same_v<Stm32G473, MCU>)) || (N == 101)
    struct Uart<N, PA, Size, ValueType, Clock, MCU> {
        using pa_t = PA;

        static inline /*constexpr */ USART_TypeDef* const mcuUart = reinterpret_cast<USART_TypeDef*>(Mcu::Stm::Address<Uart<N, PA, Size, ValueType, Clock, MCU>>::value);

        static inline constexpr uint8_t QueueLength{ Size };
        static inline constexpr bool useInterrupts{ false };

        static inline void init() {
            if constexpr (N == 1) {
                RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
                RCC->CCIPR |= 0x01 << RCC_CCIPR_USART1SEL_Pos;
            }
            else if constexpr (N == 2) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
                RCC->CCIPR |= 0x01 << RCC_CCIPR_USART2SEL_Pos;
            }
            else if constexpr (N == 3) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
                RCC->CCIPR |= 0x01 << RCC_CCIPR_USART3SEL_Pos;
            }
            else if constexpr (N == 4) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;
                RCC->CCIPR |= 0x01 << RCC_CCIPR_UART4SEL_Pos;
            }
#ifdef STM32G473xx
            else if constexpr (N == 5) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_UART5EN;
                RCC->CCIPR |= 0x01 << RCC_CCIPR_UART5SEL_Pos;
            }
#endif
            else if constexpr (N == 101) {
                RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
                RCC->CCIPR |= 0x01 << RCC_CCIPR_LPUART1SEL_Pos;
            }
            else {
                static_assert(false);
            }
            mcuUart->PRESC = 0;
            mcuUart->CR1 |= USART_CR1_FIFOEN;
            mcuUart->CR1 |= USART_CR1_RE;
            mcuUart->CR1 |= USART_CR1_TE;
            mcuUart->CR1 |= USART_CR1_UE;
        }

        template<bool Enable>
        static inline void rxEnable() {
            if constexpr (Enable) {
                mcuUart->CR1 |= USART_CR1_RE;
            }
            else {
                mcuUart->CR1 &= ~USART_CR1_RE;
            }
        }

        template<bool Enable = true>
        static inline void halfDuplex() {
            mcuUart->CR1 &= ~USART_CR1_UE;
            if constexpr (Enable) {
                mcuUart->CR3 |= USART_CR3_HDSEL;
            }
            else {
                mcuUart->CR3 &= ~USART_CR3_HDSEL;
            }
            mcuUart->CR1 |= USART_CR1_UE;
        }

        static inline void parity(const bool even) {
            mcuUart->CR1 &= ~USART_CR1_UE;
            if (even) {
                mcuUart->CR1 |= USART_CR1_PCE;
                mcuUart->CR1 &= ~USART_CR1_PS;
                mcuUart->CR1 |= USART_CR1_M0;
            }
            else {
                mcuUart->CR1 |= USART_CR1_PCE;
                mcuUart->CR1 |= USART_CR1_PS;
                mcuUart->CR1 |= USART_CR1_M0;
            }
            mcuUart->CR1 |= USART_CR1_UE;
        }
        static inline void parityOff() {
            mcuUart->CR1 &= ~USART_CR1_UE;
            mcuUart->CR1 &= ~USART_CR1_M0;
            mcuUart->CR1 &= ~USART_CR1_PCE;
            mcuUart->CR1 |= USART_CR1_UE;
        }

        static inline void rxtxswap(const bool swap) {
            mcuUart->CR1 &= ~USART_CR1_UE;
            if (swap) {
                mcuUart->CR2 |= USART_CR2_SWAP;
            }
            else {
                mcuUart->CR2 &= ~(USART_CR2_SWAP);
            }
            mcuUart->CR1 |= USART_CR1_UE;
        }

        static inline void stop(const uint8_t) {
            mcuUart->CR1 &= ~USART_CR1_UE;
            MODIFY_REG(mcuUart->CR2, USART_CR2_STOP_Msk, 0b10 << USART_CR2_STOP_Pos);
            mcuUart->CR1 |= USART_CR1_UE;
        }

        static inline void invert(const bool invert) {
            mcuUart->CR1 &= ~USART_CR1_UE;
            if (invert) {
                mcuUart->CR2 |= (USART_CR2_TXINV | USART_CR2_RXINV);
            }
            else {
                mcuUart->CR2 &= ~(USART_CR2_TXINV | USART_CR2_RXINV);
            }
            mcuUart->CR1 |= USART_CR1_UE;
        }

        static inline void baud(const uint32_t baud) {
            mcuUart->CR1 &= ~USART_CR1_UE;
            if constexpr (N == 101) {
                mcuUart->PRESC = 0b0111; // 16
                mcuUart->BRR = (16 * static_cast<Units::hertz>(Clock::config::f).value) / baud;
            }
            else {
                mcuUart->BRR = static_cast<Units::hertz>(Clock::config::f).value / baud;
            }
            mcuUart->CR1 |= USART_CR1_UE;
        }

        static inline ValueType put(const ValueType c) {
            mSendData.push_back(c);
            return c;
        }

        static inline std::optional<ValueType> get() {
            return mReceiveData.pop_front();
        }

        static inline bool isIdle() {
            return mcuUart->ISR & USART_ISR_TC;
        }

        static inline bool isTxQueueEmpty() {
            return mSendData.empty();
        }

        static inline void clear() {
            mSendData.clear();
            mReceiveData.clear();
        }


        static inline void periodic() {
            if (mcuUart->ISR & USART_ISR_TXE_TXFNF) {
                if (!mSendData.empty()) {
                    mcuUart->TDR = uint8_t(*mSendData.pop_front());
                    // Byte access of USART registers not allowed according to ref-manual
                    // may be "accessed" only by 32bit words
//                    *(uint8_t*)(&mcuUart->TDR) = *mSendData.pop_front();
                }
            }
            if (mcuUart->ISR & USART_ISR_RXNE_RXFNE) {
                if constexpr (std::is_same_v<PA, void>) {
                    mReceiveData.push_back(ValueType(mcuUart->RDR));
                }
                else {
                    PA::process(std::byte(mcuUart->RDR));
                }
            }
        }

    private:
        static inline etl::FiFo<ValueType, Size> mSendData;
        static inline etl::FiFo<ValueType, Size> mReceiveData;
    };

    template<typename PA, auto Size, typename V, typename Clock, G4xx MCU>
    struct Address<Uart<1, PA, Size, V, Clock, MCU>> {
        static inline constexpr uintptr_t value = USART1_BASE;
    };
    template<typename PA, auto Size, typename V, typename Clock, G4xx MCU>
    struct Address<Uart<2, PA, Size, V, Clock, MCU>> {
        static inline constexpr uintptr_t value = USART2_BASE;
    };
    template<typename PA, auto Size, typename V, typename Clock, G4xx MCU>
    struct Address<Uart<3, PA, Size, V, Clock, MCU>> {
        static inline constexpr uintptr_t value = USART3_BASE;
    };
    template<typename PA, auto Size, typename V, typename Clock, G4xx MCU>
    struct Address<Uart<4, PA, Size, V, Clock, MCU>> {
        static inline constexpr uintptr_t value = UART4_BASE;
    };
#ifdef STM32G473xx
    template<typename PA, auto Size, typename V, typename Clock, G4xx MCU>
    struct Address<Uart<5, PA, Size, V, Clock, MCU>> {
        static inline constexpr uintptr_t value = UART5_BASE;
    };
#endif
    template<typename PA, auto Size, typename V, typename Clock, G4xx MCU>
    struct Address<Uart<101, PA, Size, V, Clock, MCU>> {
        static inline constexpr uintptr_t value = LPUART1_BASE;
    };


}
