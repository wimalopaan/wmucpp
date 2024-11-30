#pragma once

#include "mcu/mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu/mcu_traits.h"
#include "etl/fifo.h"
#include "rcc.h"

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
                        rcc->APBRSTR2 |= RCC_APBRSTR2_USART1RST;
                        rcc->APBRSTR2 &= ~RCC_APBRSTR2_USART1RST;
                    }
                    else if constexpr(N == 2) {
                        rcc->APBRSTR1 |= RCC_APBRSTR1_USART2RST;
                        rcc->APBRSTR1 &= ~RCC_APBRSTR1_USART2RST;
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
    template<>
    struct Address<Mcu::Components::Usart<1>> {
        static inline constexpr uintptr_t value = USART1_BASE;
    };
    template<>
    struct Address<Mcu::Components::Usart<2>> {
        static inline constexpr uintptr_t value = USART2_BASE;
    };
#if defined(STM32G4)
    template<>
    struct Address<Mcu::Components::Usart<3>> {
        static inline constexpr uintptr_t value = USART3_BASE;
    };
    template<>
    struct Address<Mcu::Components::Usart<4>> {
        static inline constexpr uintptr_t value = UART4_BASE;
    };
#if defined(STM32G473xx)
    template<>
    struct Address<Mcu::Components::Usart<5>> {
        static inline constexpr uintptr_t value = UART5_BASE;
    };
#endif
    template<>
    struct Address<Mcu::Components::Usart<101>> {
        static inline constexpr uintptr_t value = LPUART1_BASE;
    };
#endif
#if defined(STM32G0B1xx)
    template<>
    struct Address<Mcu::Components::Usart<3>> {
        static inline constexpr uintptr_t value = USART3_BASE;
    };
    template<>
    struct Address<Mcu::Components::Usart<4>> {
        static inline constexpr uintptr_t value = USART4_BASE;
    };
    template<>
    struct Address<Mcu::Components::Usart<5>> {
        static inline constexpr uintptr_t value = USART5_BASE;
    };
    template<>
    struct Address<Mcu::Components::Usart<6>> {
        static inline constexpr uintptr_t value = USART6_BASE;
    };
    template<>
    struct Address<Mcu::Components::Usart<101>> {
        static inline constexpr uintptr_t value = LPUART1_BASE;
    };
    template<>
    struct Address<Mcu::Components::Usart<102>> {
        static inline constexpr uintptr_t value = LPUART2_BASE;
    };
#endif
}
