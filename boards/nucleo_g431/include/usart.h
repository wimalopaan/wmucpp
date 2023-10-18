#pragma once

#include "mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu_traits.h"
#include "fifo.h"

#include <type_traits>
#include <concepts>

namespace Mcu::Stm {
    using namespace Units::literals;

    template<uint8_t N, typename PA, uint8_t Size, typename ValueType, typename Clock, typename MCU = void> struct Uart;
    
    template<uint8_t N, typename PA, uint8_t Size, typename ValueType, typename Clock, typename MCU>
    requires (N >= 1) && (N <= 3)
    struct Uart<N, PA, Size, ValueType, Clock, MCU> {
        static inline /*constexpr */ USART_TypeDef* const mcuUart = reinterpret_cast<USART_TypeDef*>(Mcu::Stm::Address<Uart<N, PA, Size, ValueType, Clock, MCU>>::value);
        
        static inline constexpr uint8_t QueueLength{Size};
        static inline constexpr bool useInterrupts{false};
        
        static inline void init() {
            if constexpr(N == 1) {
                RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
                RCC->CCIPR |= 0x01 << RCC_CCIPR_USART1SEL_Pos;                
            }
            else if constexpr(N == 2) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
                RCC->CCIPR |= 0x01 << RCC_CCIPR_USART2SEL_Pos;                
            }
            else if constexpr(N == 3) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
                RCC->CCIPR |= 0x01 << RCC_CCIPR_USART3SEL_Pos;                
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
            if constexpr(Enable) {
                mcuUart->CR1 |= USART_CR1_RE;
            }
            else {
                mcuUart->CR1 &= ~USART_CR1_RE;
            }            
        }
        
        template<bool Enable = true>
        static inline void halfDuplex() {
            mcuUart->CR1 &= ~USART_CR1_UE;
            if constexpr(Enable) {
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
            }
            else {
                mcuUart->CR1 &= ~(USART_CR1_PCE);
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
            mcuUart->BRR = static_cast<Units::hertz>(Clock::config::f).value / baud;
        }
        
        static inline void put(const ValueType c) {
            mSendData.push_back(c);    
        }

        static inline bool isIdle() {
            return mcuUart->ISR & USART_ISR_TC;
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
                if constexpr(std::is_same_v<PA, void>) {
                    mReceiveData.push_back(mcuUart->RDR);
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

    template<typename PA, uint8_t Size, typename V, typename Clock, G4xx MCU> 
    struct Address<Uart<1, PA, Size, V, Clock, MCU>> {
        static inline constexpr uintptr_t value = USART1_BASE;
    };
    template<typename PA, uint8_t Size, typename V, typename Clock, G4xx MCU> 
    struct Address<Uart<2, PA, Size, V, Clock, MCU>> {
        static inline constexpr uintptr_t value = USART2_BASE;
    };
    template<typename PA, uint8_t Size, typename V, typename Clock, G4xx MCU> 
    struct Address<Uart<3, PA, Size, V, Clock, MCU>> {
        static inline constexpr uintptr_t value = USART3_BASE;
    };
    template<typename PA, uint8_t Size, typename V, typename Clock, G4xx MCU> 
    struct Address<Uart<4, PA, Size, V, Clock, MCU>> {
        static inline constexpr uintptr_t value = UART4_BASE;
    };
}
