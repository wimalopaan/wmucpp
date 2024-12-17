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
    namespace V4 {
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
                using dmaChComponent = Config::DmaChComponent;
                using adapter = Config::Adapter;
                using tp = Config::tp;

                struct dmaChConfig;
                using dmaChRW = Mcu::Stm::Dma::V2::Channel<dmaChComponent::number_t::value, dmaChConfig, MCU>;

                struct dmaChConfig {
                    using controller = Mcu::Stm::Dma::Controller<dmaChComponent::controller::number_t::value, MCU>;
                    using value_t = Uart::value_t;
                    static inline constexpr bool memoryIncrement = true;
                };

                static inline /*constexpr */ USART_TypeDef* const mcuUart = reinterpret_cast<USART_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Usart<N>>::value);

                static inline void reset() {
                    dmaChRW::reset();
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

                    baud<false>(Config::baudrate);

                    dmaChRW::init();

                    mcuUart->CR3 = []{
                        uint32_t cr3 = (USART_CR3_OVRDIS | USART_CR3_DMAR | USART_CR3_DMAT);
                        if constexpr(Config::halfDuplex) {
                            cr3 |= USART_CR3_HDSEL;
                        }
                        return cr3;
                    }();
                    mcuUart->CR1 = []{
                        uint32_t cr1 = 0;
                        if constexpr(Config::Rx::enable) {
                            cr1 |= USART_CR1_RE;
                        }
                        if constexpr(Config::Tx::enable) {
                            cr1 |= USART_CR1_TE;
                        }
                        if constexpr(Config::Isr::idle) {
                            cr1 |= USART_CR1_IDLEIE;
                        }
                        if constexpr(Config::Isr::txComplete) {
                            cr1 |= USART_CR1_TCIE;
                        }
                        if constexpr(Config::fifo) {
                            if constexpr (N == 1) {
                                cr1 |= USART_CR1_FIFOEN;
                            }
        #ifdef STM32G0B1xx
                            if constexpr((N == 2) || (N == 3)) {
                                cr1 |= USART_CR1_FIFOEN;
                            }
        #endif
                        }
                        return cr1;
                    }();
                    mcuUart->ICR = -1;
                    mcuUart->CR1 |= USART_CR1_UE;
                }
    #endif
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
                static inline void rxEnable() requires(std::is_same_v<dmaChRW, void>) {
                    if constexpr (Enable) {
                        mBufferHasData = false;
                        mcuUart->CR1 |= USART_CR1_RE;
                    }
                    else {
                        mcuUart->CR1 &= ~USART_CR1_RE;
                    }
                }
                template<bool Enable>
                static inline void rxEnable() requires(!std::is_same_v<dmaChRW, void>) {
                    if constexpr(Enable) {
                        mBufferHasData = false;
                        mcuUart->CR1 |= USART_CR1_RE;
                        dmaChRW::startRead(Config::Rx::size, (uint32_t)&mcuUart->RDR, mActiveReadBuffer, Uarts::Properties<N>::dmamux_rx_src);
                    }
                    else {
                        mcuUart->CR1 &= ~USART_CR1_RE;
                    }
                }

                static inline void startSend(const uint8_t n = Config::Tx::size) requires(!std::is_same_v<dmaChRW, void>) {
                    rxEnable<false>();
                    dmaChRW::startWrite(n, (uint32_t)&mcuUart->TDR, mActiveWriteBuffer, Uarts::Properties<N>::dmamux_tx_src);
                    if (mActiveWriteBuffer == &mWriteBuffer1[0]) {
                        mActiveWriteBuffer = &mWriteBuffer2[0];
                    }
                    else {
                        mActiveWriteBuffer = &mWriteBuffer1[0];
                    }
                }
                static inline auto outputBuffer() {
                    return mActiveWriteBuffer;
                }
                static inline auto readBuffer() {
                    return mActiveReadBuffer;
                }
                static inline uint16_t readCount() {
                    return *mActiveReadCount;
                }
                struct Isr {
                    static inline void onTransferComplete(const auto f) {
                        if ((mcuUart->CR1 & USART_CR1_TE) && (mcuUart->ISR & USART_ISR_TC)) {
                            mcuUart->ICR = USART_ICR_TCCF;
                            f();
                        }
                    }
                    static inline void onIdle(const auto f) requires(Config::Isr::idle && std::is_same_v<dmaChRW, void>){
                        if (mcuUart->ISR & USART_ISR_IDLE) {
                            mcuUart->ICR = USART_ICR_IDLECF;
                            f();
                        }
                    }
                    static inline void onIdle(const auto f) requires(Config::Isr::idle && !std::is_same_v<dmaChRW, void>) {
                        if (mcuUart->ISR & USART_ISR_IDLE) {
                            mcuUart->ICR = USART_ICR_IDLECF;
                            if (const uint16_t nRead = (Config::Rx::size - dmaChRW::mcuDmaChannel->CNDTR); nRead >= Config::Rx::idleMinSize) {
                                dmaChRW::reConfigure([&]{
                                    if (dmaChRW::mcuDmaChannel->CMAR == (uint32_t)&mReadBuffer1[0]) {
                                        dmaChRW::mcuDmaChannel->CMAR = (uint32_t)&mReadBuffer2[0];
                                        mActiveReadBuffer = &mReadBuffer1[0];
                                        mCount1 = nRead;
                                        mActiveReadCount = &mCount1;
                                    }
                                    else {
                                        dmaChRW::mcuDmaChannel->CMAR = (uint32_t)&mReadBuffer1[0];
                                        mActiveReadBuffer = &mReadBuffer2[0];
                                        mCount2 = nRead;
                                        mActiveReadCount = &mCount2;
                                    }
                                    dmaChRW::mcuDmaChannel->CNDTR = Config::Rx::size;
                                });
                                mBufferHasData = true;
                                f();
                            }
                        }
                    }
                };
                template<bool Disable = true>
                static inline void baud(const uint32_t baud) {
                    const auto [brr, presc] = calcBRR(baud);
                    if constexpr(Disable) {
                        mcuUart->CR1 &= ~USART_CR1_UE;
                    }
                    if constexpr ((N == 101) || (N == 102)) {
                        mcuUart->PRESC = presc;
                        mcuUart->BRR = brr;
                    }
                    else {
                        mcuUart->BRR = brr;
                    }
                    if constexpr(Disable) {
                        mcuUart->CR1 |= USART_CR1_UE;
                    }
                }
                static inline void periodic() {
                    const auto [hasData, count] = Mcu::Arm::Atomic::access([]{
                        return std::pair{std::exchange(mBufferHasData, false), readCount()};
                    });
                    if (hasData) {
                        for(uint16_t i = 0; i < count; ++i) {
                            adapter::process(mActiveReadBuffer[i]);
                        }
                    }
                }

                private:

                static inline constexpr auto calcBRR(const uint32_t baud) {
                    std::pair<uint32_t, uint32_t> brrPresc{0, 0};
                    if constexpr ((N == 101) || (N == 102)) {
                        brrPresc.first = (16 * static_cast<Units::hertz>(clock_t::config::f).value) / baud;
                        brrPresc.second = 0b0111; // 16
                    }
                    else {
                        brrPresc.first  = static_cast<Units::hertz>(clock_t::config::f).value / baud;
                    }
                    return brrPresc;
                }
                static inline std::array<storage_t, Config::Rx::size> mReadBuffer1;
                static inline std::array<storage_t, Config::Rx::size> mReadBuffer2;
                static inline storage_t* volatile mActiveReadBuffer = &mReadBuffer1[0];
                static inline volatile uint16_t mCount1 = 0;
                static inline volatile uint16_t mCount2 = 0;
                static inline volatile uint16_t* volatile mActiveReadCount = &mCount1;
                static inline bool volatile mBufferHasData = false;
                static inline std::array<storage_t, Config::Tx::size> mWriteBuffer1;
                static inline std::array<storage_t, Config::Tx::size> mWriteBuffer2;
                static inline storage_t* volatile mActiveWriteBuffer = &mWriteBuffer1[0];
            };
    }
}
