/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <type_traits>
#include <concepts>

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "units.h"
#include "concepts.h"
#include "etl/fifo.h"
#include "rcc.h"
#include "atomic.h"
#include "usarts.h"

namespace Mcu::Stm {
    using namespace Units::literals;
    namespace V2 {

        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct Uart;

        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        using LpUart = Uart<N + 100, Config, MCU>;

        template<uint8_t N, typename Config, typename MCU>
            requires (
                        ((N >= 1) && (N <= 2) && std::is_same_v<Stm32G030, MCU>) ||
                        ((N >= 1) && (N <= 6) && std::is_same_v<Stm32G0B1, MCU>) ||
                        ((N >= 1) && (N <= 3) && std::is_same_v<Stm32G431, MCU>) ||
                        ((N >= 1) && (N <= 5) && std::is_same_v<Stm32G473, MCU>) ||
                        ((N == 101)) ||
                        ((N == 102))
                    )
            struct Uart<N, Config, MCU> {
                static inline constexpr uint8_t number = N;

                using component_t = Mcu::Components::Usart<N>;

                using value_t = Config::ValueType;
                using storage_t = volatile value_t;
                using clock_t = Config::Clock;
                static inline constexpr auto size = Config::size;
                static inline constexpr auto minSize = Config::minSize;

                using dmaChRead  = Config::DmaChannelRead;
                using dmaChWrite = Config::DmaChannelWrite;
                using dmaChRW = std::conditional_t<std::is_same_v<dmaChRead, dmaChWrite>, dmaChRead, void>;

                static inline constexpr bool useDmaTCIsr = Config::useDmaTCIsr;

                static inline constexpr bool useIdleIsr = Config::useIdleIsr;
                static inline constexpr bool useRxToIsr = Config::useRxToIsr;
                static inline constexpr uint16_t rxToCount = Config::rxToCount;

                using adapter = Config::Adapter;

                using tp = Config::Debug::tp;

                static inline /*constexpr */ USART_TypeDef* const mcuUart = reinterpret_cast<USART_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Usart<N>>::value);


                static inline void reset() {
                    const auto rcc = Mcu::Stm::Address<Mcu::Components::Rcc>::value;
                    if constexpr(N == 1) {
#ifdef STM32G0
                        rcc->APBRSTR2 |= RCC_APBRSTR2_USART1RST;
                        rcc->APBRSTR2 &= ~RCC_APBRSTR2_USART1RST;
#endif
#ifdef STM32G4
                        rcc->APB2RSTR |= RCC_APB2RSTR_USART1RST;
                        rcc->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
#endif
                    }
                    else if constexpr(N == 2) {
#ifdef STM32G0
                        rcc->APBRSTR1 |= RCC_APBRSTR1_USART2RST;
                        rcc->APBRSTR1 &= ~RCC_APBRSTR1_USART2RST;
#endif
#ifdef STM32G4
                        rcc->APB1RSTR1 |= RCC_APB1RSTR1_USART2RST;
                        rcc->APB1RSTR1 &= ~RCC_APB1RSTR1_USART2RST;
#endif
                    }
#ifdef STM32G0B1xx
                    else if constexpr(N == 3) {
                        rcc->APBRSTR1 |= RCC_APBRSTR1_USART3RST;
                        rcc->APBRSTR1 &= ~RCC_APBRSTR1_USART3RST;
                    }
                    else if constexpr(N == 4) {
                        rcc->APBRSTR1 |= RCC_APBRSTR1_USART4RST;
                        rcc->APBRSTR1 &= ~RCC_APBRSTR1_USART4RST;
                    }
                    else if constexpr(N == 5) {
                        rcc->APBRSTR1 = RCC_APBRSTR1_USART5RST;
                        rcc->APBRSTR1 &= ~RCC_APBRSTR1_USART5RST;
                    }
                    else if constexpr(N == 6) {
                        rcc->APBRSTR1 |= RCC_APBRSTR1_USART6RST;
                        rcc->APBRSTR1 &= ~RCC_APBRSTR1_USART6RST;
                    }
                    else if constexpr(N == 101) {
                        rcc->APBRSTR1 |= RCC_APBRSTR1_LPUART1RST;
                        rcc->APBRSTR1 &= ~RCC_APBRSTR1_LPUART1RST;
                    }
                    else if constexpr(N == 102) {
                        rcc->APBRSTR1 |= RCC_APBRSTR1_LPUART2RST;
                        rcc->APBRSTR1 &= ~RCC_APBRSTR1_LPUART2RST;
                    }
#endif
                    else {
                        static_assert(false);
                    }
                }
#ifdef STM32G0
                static inline void init() {
                    if constexpr (N == 1) {
                        RCC->APBENR2 |= RCC_APBENR2_USART1EN;
                        RCC->CCIPR |= 0x01 << RCC_CCIPR_USART1SEL_Pos;
                    }
                    else if constexpr (N == 2) {
                        RCC->APBENR1 |= RCC_APBENR1_USART2EN;
#ifdef STM32G0B1xx
                        RCC->CCIPR |= 0x01 << RCC_CCIPR_USART2SEL_Pos;
#endif
                    }
#ifdef STM32G0B1xx
                    else if constexpr (N == 3) {
                        RCC->APBENR1 |= RCC_APBENR1_USART3EN;
                        RCC->CCIPR |= 0x01 << RCC_CCIPR_USART3SEL_Pos;
                    }
                    else if constexpr (N == 4) {
                        RCC->APBENR1 |= RCC_APBENR1_USART4EN;
                    }
                    else if constexpr (N == 5) {
                        RCC->APBENR1 |= RCC_APBENR1_USART5EN;
                    }
                    else if constexpr (N == 6) {
                        RCC->APBENR1 |= RCC_APBENR1_USART6EN;
                    }
                    else if constexpr (N == 101) {
                        RCC->APBENR1 |= RCC_APBENR1_LPUART1EN;
                        RCC->CCIPR |= 0x01 << RCC_CCIPR_LPUART1SEL_Pos;
                    }
                    else if constexpr (N == 102) {
                        RCC->APBENR1 |= RCC_APBENR1_LPUART2EN;
                        RCC->CCIPR |= 0x01 << RCC_CCIPR_LPUART2SEL_Pos;
                    }
#endif
                    else {
                        static_assert(false);
                    }
                    if constexpr(N == 1) {
                        mcuUart->CR1 |= USART_CR1_FIFOEN;
                    }
#ifdef STM32G0B1xx
                    if constexpr((N == 2) || (N == 3)) {
                        mcuUart->CR1 |= USART_CR1_FIFOEN;
                    }
#endif
                    if constexpr(useRxToIsr) {
                        static_assert((N >= 1) && (N <= 3));
                        static_assert(rxToCount > 8);
                        mcuUart->CR2 |= USART_CR2_RTOEN;
                        mcuUart->CR1 |= USART_CR1_RTOIE;
                        mcuUart->RTOR = rxToCount;
                    }
                    if constexpr(useIdleIsr) {
                        mcuUart->CR1 |= USART_CR1_IDLEIE;
                    }
                    if constexpr(std::is_same_v<dmaChRead, dmaChWrite>) {
                        dmaSetupWrite(size, (uint32_t)&mWriteBuffer1[0]);
                    }
                    else {
                        if constexpr(!std::is_same_v<dmaChRead, void>) {
                            dmaChRead::init();
                            dmaChRead::template msize<value_t>();
                            dmaChRead::template psize<value_t>();
                            dmaChRead::mcuDmaChannel->CNDTR = size;
                            dmaChRead::mcuDmaChannel->CCR |= DMA_CCR_MINC;
                            dmaChRead::mcuDmaChannel->CPAR = (uint32_t)&mcuUart->RDR;
                            dmaChRead::mcuDmaChannel->CMAR = (uint32_t)&mReadBuffer1[0];
                            MODIFY_REG(dmaChRead::mcuDmaMux->CCR, DMAMUX_CxCR_DMAREQ_ID_Msk,
                                       Uarts::Properties<N>::dmamux_rx_src << DMAMUX_CxCR_DMAREQ_ID_Pos);
                            dmaChRead::enable(true);
                        }
                        if constexpr (!std::is_same_v<dmaChWrite, void>) {
                            dmaChWrite::init();
                            dmaChWrite::template msize<value_t>();
                            dmaChWrite::template psize<value_t>();
                            dmaChWrite::mcuDmaChannel->CNDTR = size;
                            dmaChWrite::mcuDmaChannel->CCR |= DMA_CCR_MINC;
                            dmaChWrite::mcuDmaChannel->CCR |= DMA_CCR_DIR;
                            if constexpr(useDmaTCIsr) {
                                dmaChWrite::mcuDmaChannel->CCR |= DMA_CCR_TCIE;
                            }
                            dmaChWrite::mcuDmaChannel->CPAR = (uint32_t)&mcuUart->TDR;
                            dmaChWrite::mcuDmaChannel->CMAR = (uint32_t)&mWriteBuffer1[0];
                            MODIFY_REG(dmaChWrite::mcuDmaMux->CCR, DMAMUX_CxCR_DMAREQ_ID_Msk,
                                       Uarts::Properties<N>::dmamux_tx_src << DMAMUX_CxCR_DMAREQ_ID_Pos);
                            // dmaChWrite::enable(true);
                        }
                        if constexpr(std::is_same_v<dmaChRead, void> && std::is_same_v<dmaChWrite, void>) {
                            static_assert(false);
                        }
                    }

                    mcuUart->CR3 |= USART_CR3_OVRDIS;
                    mcuUart->CR3 |= USART_CR3_DMAR;
                    mcuUart->CR3 |= USART_CR3_DMAT;

                    mcuUart->CR1 |= USART_CR1_RE;
                    mcuUart->CR1 |= USART_CR1_TE;
                    mcuUart->CR1 |= USART_CR1_UE;
                }
    #endif
                static inline void dmaSetupWrite(const size_t s, const uint32_t adr) requires(std::is_same_v<dmaChRead, dmaChWrite>) {
                    using dmaChRW = dmaChWrite;
                    dmaChRW::init();
                    dmaChRW::template msize<value_t>();
                    dmaChRW::template psize<value_t>();
                    dmaChRW::mcuDmaChannel->CNDTR = s;
                    dmaChRW::mcuDmaChannel->CCR |= DMA_CCR_MINC;
                    dmaChRW::mcuDmaChannel->CCR |= DMA_CCR_DIR;
                    if constexpr(useDmaTCIsr) {
                        dmaChRW::mcuDmaChannel->CCR |= DMA_CCR_TCIE;
                    }
                    else {
                        dmaChRW::mcuDmaChannel->CCR &= ~DMA_CCR_TCIE;
                    }

                    dmaChRW::mcuDmaChannel->CPAR = (uint32_t)&mcuUart->TDR;
                    dmaChRW::mcuDmaChannel->CMAR = adr;
                    MODIFY_REG(dmaChRW::mcuDmaMux->CCR, DMAMUX_CxCR_DMAREQ_ID_Msk,
                               Uarts::Properties<N>::dmamux_tx_src << DMAMUX_CxCR_DMAREQ_ID_Pos);
                }
                static inline void dmaSetupRead(const size_t s) requires(std::is_same_v<dmaChRead, dmaChWrite>) {
                    // using dmaChRW = dmaChRead;
                    dmaChRW::init();
                    dmaChRW::template msize<value_t>();
                    dmaChRW::template psize<value_t>();
                    dmaChRW::mcuDmaChannel->CNDTR = s;
                    dmaChRW::mcuDmaChannel->CCR |= DMA_CCR_MINC;
                    dmaChRW::mcuDmaChannel->CCR &= ~DMA_CCR_DIR;
                    dmaChRW::mcuDmaChannel->CPAR = (uint32_t)&mcuUart->RDR;
                    dmaChRW::mcuDmaChannel->CMAR = (uint32_t)&mReadBuffer1[0];
                    MODIFY_REG(dmaChRW::mcuDmaMux->CCR, DMAMUX_CxCR_DMAREQ_ID_Msk,
                               Uarts::Properties<N>::dmamux_rx_src << DMAMUX_CxCR_DMAREQ_ID_Pos);
                }
                static inline void dmaSetupRead2(const size_t s) requires(std::is_same_v<dmaChRead, dmaChWrite>) {
                    // using dmaChRW = dmaChRead;
                    dmaChRW::mcuDmaChannel->CNDTR = s;
                    dmaChRW::mcuDmaChannel->CCR &= ~DMA_CCR_DIR;
                    dmaChRW::mcuDmaChannel->CPAR = (uint32_t)&mcuUart->RDR;
                    dmaChRW::mcuDmaChannel->CMAR = (uint32_t)&mReadBuffer1[0];
                    MODIFY_REG(dmaChRW::mcuDmaMux->CCR, DMAMUX_CxCR_DMAREQ_ID_Msk,
                               Uarts::Properties<N>::dmamux_rx_src << DMAMUX_CxCR_DMAREQ_ID_Pos);
                }

                static inline void onTransferComplete(const auto f) {
                    if ((mcuUart->CR1 & USART_CR1_TE) && (mcuUart->ISR & USART_ISR_TC)) {
                        mcuUart->ICR = USART_ICR_TCCF;
                        f();
                    }
                }
                static inline void onIdle(const auto f) {
                    if (mcuUart->ISR & USART_ISR_IDLE) {
                        mcuUart->ICR = USART_ICR_IDLECF;
                        f();
                    }
                }
                static inline void onParityError(const auto f) {
                    if (mcuUart->ISR & USART_ISR_PE) {
                        mcuUart->ICR = USART_ICR_PECF;
                        f();
                    }
                }
                static inline void onParityGood(const auto f) {
                    if (mcuUart->ISR & USART_ISR_PE) {
                        mcuUart->ICR = USART_ICR_PECF;
                    }
                    else {
                        f();
                    }
                }
                static inline void onIdleWithDma(const auto f) requires(!std::is_same_v<dmaChRead, void> && std::is_same_v<dmaChWrite, void>) {
                    if (mcuUart->ISR & USART_ISR_IDLE) {
                        mcuUart->ICR = USART_ICR_IDLECF;
                        if (const uint16_t nRead = (size - dmaChRead::mcuDmaChannel->CNDTR); nRead >= minSize) {
                            mCount = nRead;
                            f();
                        }
                    }
                }
                static inline void onIdleWithDma(const auto f) requires(std::is_same_v<dmaChRead, dmaChWrite>) {
                    if (mcuUart->ISR & USART_ISR_IDLE) {
                        mcuUart->ICR = USART_ICR_IDLECF;
                        if constexpr(std::is_same_v<dmaChRead, dmaChWrite>) {
                            if (const uint16_t nRead = (size - dmaChRead::mcuDmaChannel->CNDTR); nRead > minSize) {
                                mCount = nRead;
                                f();
                            }
                        }
                    }
                }
                template<bool Enable>
                static inline void txEnable() {
                    if constexpr (Enable) {
                        mcuUart->CR1 |= USART_CR1_TE;
                    }
                    else {
                        mcuUart->CR1 &= ~USART_CR1_TE;
                    }
                }
                template<bool Enable>
                static inline void rxEnable() {
                    if constexpr (Enable) {
                        mBufferHasData = false;
                        mcuUart->CR1 |= USART_CR1_RE;
                    }
                    else {
                        mcuUart->CR1 &= ~USART_CR1_RE;
                    }
                }
                static inline void rxClear() {
                    mcuUart->RQR = USART_RQR_RXFRQ;
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
                template<bool Enable = true>
                static inline void enableTCIsr() {
                    mcuUart->CR1 &= ~USART_CR1_UE;
                    if constexpr(Enable) {
                        mcuUart->CR1 |= USART_CR1_TCIE;
                    }
                    else {
                        mcuUart->CR1 &= ~USART_CR1_TCIE;
                    }
                    mcuUart->CR1 |= USART_CR1_UE;
                }
                static inline void dmaReenable(const auto f) requires(std::is_same_v<dmaChRead, dmaChWrite>){
                    using dmaChRW = dmaChRead;
                    dmaChRW::reenable(f);
                }
                static inline void startSend(const uint8_t n = size) {
                    if constexpr(std::is_same_v<dmaChRead, dmaChWrite>) {
                        using dmaChRW = dmaChRead;
                        dmaChRW::enable(false);
                        dmaChRW::clearTransferCompleteIF();
                        dmaSetupWrite(n, (uint32_t)mActiveWriteBuffer);
                        if (mActiveWriteBuffer == &mWriteBuffer1[0]) {
                            mActiveWriteBuffer = &mWriteBuffer2[0];
                        }
                        else {
                            mActiveWriteBuffer = &mWriteBuffer1[0];
                        }
                        dmaChRW::enable(true);
                    }
                    else {
                        dmaChWrite::reenable([&]{
                            dmaChWrite::mcuDmaChannel->CNDTR = n;
                            dmaChWrite::mcuDmaChannel->CMAR = (uint32_t)mActiveWriteBuffer;
                            if (mActiveWriteBuffer == &mWriteBuffer1[0]) {
                                mActiveWriteBuffer = &mWriteBuffer2[0];
                            }
                            else {
                                mActiveWriteBuffer = &mWriteBuffer1[0];
                            }
                        });
                    }
                }
                static inline void dmaDisable() {
                    dmaChWrite::enable(false);
                }
                static inline auto outputBuffer() {
                    return mActiveWriteBuffer;
                }
                static inline auto readBuffer() {
                    return mActiveReadBuffer;
                }
                static inline uint16_t readCount() {
                    return mCount;
                }
                static inline void isr() requires(useRxToIsr || useIdleIsr) {
                    auto reprogDma = []{
                        if constexpr(std::is_same_v<dmaChRead, dmaChWrite>) {
                            if (const uint16_t nRead = (size - dmaChRead::mcuDmaChannel->CNDTR); nRead >= minSize) {
                                mCount = nRead;
                                mBufferHasData = true;
                            }
                        }
                        else {
                            if (const uint16_t nRead = (size - dmaChRead::mcuDmaChannel->CNDTR); nRead >= minSize) {
                                mCount = nRead;
                                dmaChRead::reenable([&]{
                                    if (dmaChRead::mcuDmaChannel->CMAR == (uint32_t)&mReadBuffer1[0]) {
                                        dmaChRead::mcuDmaChannel->CMAR = (uint32_t)&mReadBuffer2[0];
                                        mActiveReadBuffer = &mReadBuffer1[0];
                                    }
                                    else {
                                        dmaChRead::mcuDmaChannel->CMAR = (uint32_t)&mReadBuffer1[0];
                                        mActiveReadBuffer = &mReadBuffer2[0];
                                    }
                                    dmaChRead::mcuDmaChannel->CNDTR = size;
                                });
                                mBufferHasData = true;
                            }
                        }
                    };
                    if (mcuUart->ISR & USART_ISR_RTOF) {
                        mcuUart->ICR = USART_ICR_RTOCF;
                        reprogDma();
                    }
                    if (mcuUart->ISR & USART_ISR_IDLE) {
                        mcuUart->ICR = USART_ICR_IDLECF;
                        reprogDma();
                    }
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
                static inline void baud(const uint32_t baud) {
                    mcuUart->CR1 &= ~USART_CR1_UE;
                    if constexpr ((N == 101) || (N == 102)) {
                        mcuUart->PRESC = 0b0111; // 16
                        mcuUart->BRR = (16 * static_cast<Units::hertz>(clock_t::config::f).value) / baud;
                    }
                    else {
                        mcuUart->BRR = static_cast<Units::hertz>(clock_t::config::f).value / baud;
                    }
                    mcuUart->CR1 |= USART_CR1_UE;
                }
                static inline bool isIdle() {
                    return mcuUart->ISR & USART_ISR_TC;
                }
                static inline void onHasData(auto f) {
                    __disable_irq();
                    const bool hasData = std::exchange(mBufferHasData, false);
                    __enable_irq();
                    if (hasData) {
                        f();
                    }
                }
                static inline void periodic() {
                    onHasData([]{
                        adapter::reset();
                        analyze();
                    });
                    // __disable_irq();
                    // const bool hasData = std::exchange(mBufferHasData, false);
                    // __enable_irq();
                    // if (hasData) {
                    //     tp::set();
                    //     adapter::reset();
                    //     analyze();
                    //     tp::reset();
                    // }
                }

                private:
                static inline void analyze() {
                    for(uint16_t i = 0; i < mCount; ++i) {
                        adapter::process(mActiveReadBuffer[i]);
                    }
                }

                static inline std::array<storage_t, size> mReadBuffer1;
                static inline std::array<storage_t, size> mReadBuffer2;
                static inline storage_t* volatile mActiveReadBuffer = &mReadBuffer1[0];
                static inline volatile uint16_t mCount = 0;
                static inline bool volatile mBufferHasData = false;
                static inline std::array<storage_t, size> mWriteBuffer1;
                static inline std::array<storage_t, size> mWriteBuffer2;
                static inline storage_t* volatile mActiveWriteBuffer = &mWriteBuffer1[0];
            };
    }
    inline
    namespace V1 {

    template<uint8_t N, typename PA, auto Size, typename ValueType, typename Clock, typename MCU = DefaultMcu>
    struct Uart;

    template<uint8_t N, typename PA, auto Size, typename ValueType, typename Clock, typename MCU = DefaultMcu>
    using LpUart = Uart<N + 100, PA, Size, ValueType, Clock, MCU>;

    template<uint8_t N, typename PA, auto Size, typename ValueType, typename Clock, typename MCU>
        requires (
                    ((N >= 1) && (N <= 2) && std::is_same_v<Stm32G030, MCU>) ||
                    ((N >= 1) && (N <= 6) && std::is_same_v<Stm32G0B1, MCU>) ||
                    ((N >= 1) && (N <= 3) && std::is_same_v<Stm32G431, MCU>) ||
                    ((N >= 1) && (N <= 5) && std::is_same_v<Stm32G473, MCU>)) || (N == 101)
        struct Uart<N, PA, Size, ValueType, Clock, MCU> {
            using pa_t = PA;

            static inline /*constexpr */ USART_TypeDef* const mcuUart = reinterpret_cast<USART_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Usart<N>>::value);

            static inline constexpr uint8_t QueueLength{ Size };
            static inline constexpr bool useInterrupts{ false };

#ifdef STM32G4
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
#endif
#ifdef STM32G0
            static inline void init() {
                if constexpr (N == 1) {
                    RCC->APBENR2 |= RCC_APBENR2_USART1EN;
                    RCC->CCIPR |= 0x01 << RCC_CCIPR_USART1SEL_Pos;
                }
                else if constexpr (N == 2) {
                    RCC->APBENR1 |= RCC_APBENR1_USART2EN;
#ifdef STM32G0B1xx
                    RCC->CCIPR |= 0x01 << RCC_CCIPR_USART2SEL_Pos;
#endif
                }
#ifdef STM32G0B1xx
                else if constexpr (N == 3) {
                    RCC->APBENR1 |= RCC_APBENR1_USART3EN;
                    RCC->CCIPR |= 0x01 << RCC_CCIPR_USART3SEL_Pos;
                }
                else if constexpr (N == 4) {
                    RCC->APBENR1 |= RCC_APBENR1_USART4EN;
                }
                else if constexpr (N == 5) {
                    RCC->APBENR1 |= RCC_APBENR1_USART5EN;
                }
                else if constexpr (N == 6) {
                    RCC->APBENR1 |= RCC_APBENR1_USART6EN;
                }
                else if constexpr (N == 101) {
                    RCC->APBENR1 |= RCC_APBENR1_LPUART1EN;
                    RCC->CCIPR |= 0x01 << RCC_CCIPR_LPUART1SEL_Pos;
                }
                else if constexpr (N == 102) {
                    RCC->APBENR1 |= RCC_APBENR1_LPUART2EN;
                    RCC->CCIPR |= 0x01 << RCC_CCIPR_LPUART2SEL_Pos;
                }
#endif
                else {
                    static_assert(false);
                }
                if constexpr(N == 1) {
                    mcuUart->CR1 |= USART_CR1_FIFOEN;
                }
#ifdef STM32G0B1xx
                if constexpr((N == 2) || (N == 3)) {
                    mcuUart->CR1 |= USART_CR1_FIFOEN;
                }
#endif
                mcuUart->CR3 |= USART_CR3_OVRDIS;
                mcuUart->CR1 |= USART_CR1_RE;
                mcuUart->CR1 |= USART_CR1_TE;
                mcuUart->CR1 |= USART_CR1_UE;
            }
#endif
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
                if constexpr ((N == 101) || (N == 102)) {
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
    }

}
