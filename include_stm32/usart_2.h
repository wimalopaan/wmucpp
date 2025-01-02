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
#include "usart_2_reflection.h"
#include "dma_2.h"

#if 0
struct UartConfig {
    using Clock = clock;
    using ValueType = uint8_t;
    using DmaChComponent = dmaChComponent;
    using Adapter = ProtocolAdapter<Serial>;                                 // or: void
    static inline constexpr bool rxtxswap = false;                           // optional
    static inline constexpr bool fifo = true;
    static inline constexpr bool invert = true;                              // optional
    static inline constexpr auto parity = Mcu::Stm::Uarts::Parity::Even;     // optional
    static inline constexpr Mode mode = Mode::HalfDuplex; // TxOnly, RxOnly, HalfDuplex, FullDuplex
    static inline constexpr uint32_t baudrate = 115'200;
    struct Rx {
        static inline constexpr bool enable = false;
        static inline constexpr size_t size = 256;
        static inline constexpr size_t idleMinSize = 8;
    };
    struct Tx {
        static inline constexpr bool enable = true;
        static inline constexpr size_t size = 64;
    };
    struct Isr {
        static inline constexpr bool idle = true;
        static inline constexpr bool txComplete = true;
    };
    using tp = Serial::tp;
};
#endif

namespace Mcu::Stm {
    using namespace Units::literals;
    namespace V4 {
        namespace detail {
            template<typename Ch_RW, typename Ch_R, typename Ch_W, typename Config>
            struct DualChannel;

            template<typename Ch_RW, typename Config>
            struct DualChannel<Ch_RW, void, void, Config> {
                using value_t = Config::value_t;
                struct dmaChConfig;
                using dmaChRW = Mcu::Stm::Dma::V2::Channel<Ch_RW::number_t::value, dmaChConfig>;
                struct dmaChConfig {
                    using controller = Mcu::Stm::Dma::Controller<Ch_RW::controller::number_t::value>;
                    using value_t = DualChannel::value_t;
                    static inline constexpr bool memoryIncrement = true;
                };
                static inline void init() {
                    dmaChRW::init();
                }
                static inline void reset() {
                    dmaChRW::reset();
                }
                static inline void startRead(const size_t size, const uint32_t pAdr, volatile value_t* mAdr, uint8_t mux) {
                    dmaChRW::startRead(size, pAdr, mAdr, mux);
                }
                static inline void startWrite(const size_t size, const uint32_t pAdr, volatile value_t* mAdr, uint8_t mux) {
                    dmaChRW::startWrite(size, pAdr, mAdr, mux);
                }
                static inline uint32_t counter() {
                    return dmaChRW::counter();
                }
                static inline void memoryAddress(volatile value_t* adr) {
                    dmaChRW::memoryAddress(adr);
                }
                static inline volatile value_t* memoryAddress() {
                    return dmaChRW::memoryAddress();
                }
                // static inline void enable(const bool on = true) {

                // }
                static inline void reConfigure(const auto f) {
                    dmaChRW::reConfigure(f);
                }
                static inline void size(const uint32_t s) {
                    dmaChRW::size(s);
                }
            };
            template<typename Ch_R, typename Ch_W, typename Config>
            struct DualChannel<void, Ch_R, Ch_W, Config> {
                using value_t = Config::value_t;
                struct dmaRConfig {
                    using controller = Mcu::Stm::Dma::Controller<Ch_R::controller::number_t::value>;
                    using value_t = DualChannel::value_t;
                    static inline constexpr bool memoryIncrement = true;
                };
                using dmaChR = Mcu::Stm::Dma::V2::Channel<Ch_R::number_t::value, dmaRConfig>;
                struct dmaWConfig {
                    using controller = Mcu::Stm::Dma::Controller<Ch_W::controller::number_t::value>;
                    using value_t = DualChannel::value_t;
                    static inline constexpr bool memoryIncrement = true;
                };
                using dmaChW = Mcu::Stm::Dma::V2::Channel<Ch_W::number_t::value, dmaWConfig>;

                static inline void init() {
                    dmaChR::init();
                    dmaChW::init();
                }
                static inline void reset() {
                    dmaChR::reset();
                    dmaChW::reset();
                }
                static inline void startRead(const size_t size, const uint32_t pAdr, volatile value_t* mAdr, uint8_t mux) {
                    dmaChR::startRead(size, pAdr, mAdr, mux);
                }
                static inline void startWrite(const size_t size, const uint32_t pAdr, volatile value_t* mAdr, uint8_t mux) {
                    dmaChW::startWrite(size, pAdr, mAdr, mux);
                }
                template<bool Read = true>
                static inline uint32_t counter() {
                    if constexpr(Read) {
                        return dmaChR::counter();
                    }
                    else {
                        return dmaChW::counter();
                    }
                }
                template<bool Read = true>
                static inline void memoryAddress(volatile value_t* adr) {
                    if constexpr(Read) {
                        dmaChR::memoryAddress(adr);
                    }
                    else {
                        dmaChW::memoryAddress(adr);
                    }
                }
                template<bool Read = true>
                static inline volatile value_t* memoryAddress() {
                    if constexpr(Read) {
                        return dmaChR::memoryAddress();
                    }
                    else {
                        return dmaChW::memoryAddress();
                    }
                }
                // static inline void enable(const bool on = true) {
                // }
                template<bool Read = true>
                static inline void reConfigure(const auto f) {
                    if constexpr(Read) {
                        return dmaChR::reConfigure(f);
                    }
                    else {
                        return dmaChW::reConfigure(f);
                    }
                }
                template<bool Read = true>
                static inline void size(const uint32_t s) {
                    if constexpr(Read) {
                        return dmaChR::size(s);
                    }
                    else {
                        return dmaChW::size(s);
                    }
                }
            };
        }

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
            using adapter = detail::getAdapter_t<Config>;
            using tp = detail::getTp_t<Config>;

            static inline constexpr bool hasTx = detail::hasTx<Config>;
            static inline constexpr bool hasRx = detail::hasRx<Config>;

            static_assert(((Config::mode == Uarts::Mode::RxOnly) && hasRx && !hasTx) || ((Config::mode != Uarts::Mode::RxOnly) && hasTx));
            static_assert(((Config::mode == Uarts::Mode::TxOnly) && hasTx && !hasRx) || ((Config::mode != Uarts::Mode::TxOnly) && hasRx));

            static inline constexpr bool hasRWDma = detail::hasDmaComponent<Config>;

            using rwDmaComponent_t = std::conditional_t<(Config::mode != Uarts::Mode::FullDuplex), detail::getDmaComponent_t<Config>, void>;
            using rxDmaComponent_t = std::conditional_t<(Config::mode == Uarts::Mode::FullDuplex), detail::getDmaComponent_t<detail::getRx_t<Config>>, void>;
            using txDmaComponent_t = std::conditional_t<(Config::mode == Uarts::Mode::FullDuplex), detail::getDmaComponent_t<detail::getTx_t<Config>>, void>;

            static inline constexpr bool useDma = !(std::is_same_v<rwDmaComponent_t, void> && std::is_same_v<rxDmaComponent_t, void> && std::is_same_v<txDmaComponent_t, void>);

            using rx_buffer_t = std::conditional_t<(Config::mode != Uarts::Mode::TxOnly), std::array<storage_t, detail::getSize_v<detail::getRx_t<Config>>>, struct Dummy>;
            using tx_buffer_t = std::conditional_t<(Config::mode != Uarts::Mode::RxOnly), std::array<storage_t, detail::getSize_v<detail::getTx_t<Config>>>, struct Dummy>;

            static inline constexpr bool useSingleTxBuffer = detail::getSingleBuffer_v<detail::getTx_t<Config>>;
            using tx_buffer2_t = std::conditional_t<useSingleTxBuffer, struct Dummy, tx_buffer_t>;

            using tx_fifo_t = std::conditional_t<useDma, struct Dummy, etl::FiFo<value_t, detail::getSize_v<detail::getTx_t<Config>>>>;

            struct dmaChConfig {
                using value_t = Uart::value_t;
                static inline constexpr bool memoryIncrement = true;
            };
            using dmaChRW = detail::DualChannel<rwDmaComponent_t, rxDmaComponent_t, txDmaComponent_t, dmaChConfig>;

            static inline /*constexpr */ USART_TypeDef* const mcuUart = reinterpret_cast<USART_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Usart<N>>::value);

            static inline void reset() {
                if constexpr(useDma) {
                    dmaChRW::reset();
                }
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

                if constexpr(useDma) {
                    dmaChRW::init();
                    if constexpr(detail::getEnable_v<detail::getRx_t<Config>>) {
                        dmaChRW::startRead(Config::Rx::size, (uint32_t)&mcuUart->RDR, mActiveReadBuffer, Uarts::Properties<N>::dmamux_rx_src);
                    }
                }

                mcuUart->CR3 = []consteval{
                    uint32_t cr3 = (USART_CR3_OVRDIS | USART_CR3_DMAR | USART_CR3_DMAT);
                    if constexpr(Config::mode == Uarts::Mode::HalfDuplex) {
                        cr3 |= USART_CR3_HDSEL;
                    }
                    return cr3;
                }();
                mcuUart->CR2 = []consteval{
                    uint32_t cr2 = 0;
                    if constexpr(detail::getInvert_v<Config>) {
                        cr2 |= (USART_CR2_TXINV | USART_CR2_RXINV);
                    }
                    if constexpr(detail::getSwap_v<Config>) {
                        cr2 |= USART_CR2_SWAP;
                    }
                    return cr2;
                }();
                mcuUart->CR1 = []consteval{
                    uint32_t cr1 = 0;
                    if constexpr(detail::getEnable_v<detail::getRx_t<Config>>) {
                        cr1 |= USART_CR1_RE;
                    }
                    if constexpr(detail::getEnable_v<detail::getTx_t<Config>>) {
                        cr1 |= USART_CR1_TE;
                    }
                    if constexpr(detail::getIdle_v<detail::getIsr_t<Config>>) {
                        cr1 |= USART_CR1_IDLEIE;
                    }
                    if constexpr(detail::getTxComplete_v<detail::getIsr_t<Config>>) {
                        cr1 |= USART_CR1_TCIE;
                    }
                    if constexpr(detail::getParity_v<Config> == Uarts::Parity::Even) {
                        cr1 |= USART_CR1_PCE;
                        cr1 &= ~USART_CR1_PS;
                        cr1 |= USART_CR1_M0;
                    }
                    else if constexpr(detail::getParity_v<Config> == Uarts::Parity::Odd) {
                        cr1 |= USART_CR1_PCE;
                        cr1 |= USART_CR1_PS;
                        cr1 |= USART_CR1_M0;
                    }
                    if constexpr(detail::getFifo_v<Config>) {
                        if constexpr (N == 1) {
                            cr1 |= USART_CR1_FIFOEN;
                        }
#ifdef STM32G0B1xx
                        else if constexpr((N == 2) || (N == 3)) {
                            cr1 |= USART_CR1_FIFOEN;
                        }
                        else if constexpr((N == 101) || (N == 102)) {
                            cr1 |= USART_CR1_FIFOEN;
                        }
#endif
                        else {
                            static_assert(false);
                        }
                    }
                    return cr1;
                }();
                mcuUart->ICR = -1;
                mcuUart->CR1 |= USART_CR1_UE;
            }
#endif
            static inline void onParityError(const auto f) requires(Config::mode != Uarts::Mode::TxOnly) {
                if (mcuUart->ISR & USART_ISR_PE) {
                    mcuUart->ICR = USART_ICR_PECF;
                    f();
                }
            }
            static inline void onParityGood(const auto f) requires(Config::mode != Uarts::Mode::TxOnly) {
                if (mcuUart->ISR & USART_ISR_PE) {
                    mcuUart->ICR = USART_ICR_PECF;
                }
                else {
                    f();
                }
            }
            static inline void clearAll() {
                mcuUart->ICR = -1;
            }
            template<bool Enable>
            static inline void txEnable() requires(Config::mode != Uarts::Mode::RxOnly) {
                if constexpr (Enable) {
                    mcuUart->CR1 |= USART_CR1_TE;
                }
                else {
                    mcuUart->CR1 &= ~USART_CR1_TE;
                }
            }
            template<bool Enable>
            static inline void rxEnable()
                    requires(std::is_same_v<dmaChRW, void> && (Config::mode != Uarts::Mode::TxOnly)) {
                if constexpr (Enable) {
                    mBufferHasData = false;
                    mcuUart->CR1 |= USART_CR1_RE;
                }
                else {
                    mcuUart->CR1 &= ~USART_CR1_RE;
                }
            }
            template<bool Enable>
            static inline void rxEnable()
                    requires(!std::is_same_v<dmaChRW, void> && (Config::mode != Uarts::Mode::TxOnly)) {
                if constexpr(Enable) {
                    mBufferHasData = false;
                    mcuUart->CR1 |= USART_CR1_RE;
                    dmaChRW::startRead(Config::Rx::size, (uint32_t)&mcuUart->RDR, mActiveReadBuffer, Uarts::Properties<N>::dmamux_rx_src);
                }
                else {
                    mcuUart->CR1 &= ~USART_CR1_RE;
                }
            }
            static inline void startSend(const uint8_t n = Config::Tx::size)
                    requires(!std::is_same_v<dmaChRW, void> && (Config::mode != Uarts::Mode::RxOnly)) {
                if constexpr(Config::mode == Uarts::Mode::HalfDuplex) {
                    rxEnable<false>();
                }
                dmaChRW::startWrite(n, (uint32_t)&mcuUart->TDR, mActiveWriteBuffer, Uarts::Properties<N>::dmamux_tx_src);
                if constexpr(!useSingleTxBuffer) {
                    if (mActiveWriteBuffer == &mWriteBuffer1[0]) {
                        mActiveWriteBuffer = &mWriteBuffer2[0];
                    }
                    else {
                        mActiveWriteBuffer = &mWriteBuffer1[0];
                    }
                }
            }
            static inline auto fillSendBuffer(const auto f)
                    requires((Config::mode != Uarts::Mode::RxOnly) && (useSingleTxBuffer)) {
                const uint16_t n = f(mWriteBuffer1);
                startSend(n);
            }
            static inline auto outputBuffer() requires(Config::mode != Uarts::Mode::RxOnly) {
                return mActiveWriteBuffer;
            }
            static inline auto readBuffer(const auto f)
                    requires((Config::mode != Uarts::Mode::TxOnly) && (std::is_same_v<adapter, void>)) {
                f(std::span{mActiveReadBuffer, (size_t)*mActiveReadCount});
            }
            static inline auto readBuffer()
                    requires((Config::mode != Uarts::Mode::TxOnly) && (std::is_same_v<adapter, void>)) {
                return mActiveReadBuffer;
            }
            static inline uint16_t readCount() requires(Config::mode != Uarts::Mode::TxOnly) {
                return *mActiveReadCount;
            }
            struct Isr {
                static inline void onTransferComplete(const auto f)
                        requires(Config::mode != Uarts::Mode::RxOnly) {
                    if ((mcuUart->CR1 & USART_CR1_TE) && (mcuUart->ISR & USART_ISR_TC)) {
                        mcuUart->ICR = USART_ICR_TCCF;
                        f();
                        if constexpr(Config::mode == Uarts::Mode::HalfDuplex) {
                            rxEnable<true>();
                        }
                    }
                }
                static inline void onIdle(const auto f)
                        requires(Config::Isr::idle && std::is_same_v<dmaChRW, void> && (Config::mode != Uarts::Mode::TxOnly)){
                    if (mcuUart->ISR & USART_ISR_IDLE) {
                        mcuUart->ICR = USART_ICR_IDLECF;
                        f();
                    }
                }
                static inline void onIdle(const auto f)
                        requires(Config::Isr::idle && !std::is_same_v<dmaChRW, void> && (Config::mode != Uarts::Mode::TxOnly)) {
                    if (mcuUart->ISR & USART_ISR_IDLE) {
                        mcuUart->ICR = USART_ICR_IDLECF;
                        if (const uint16_t nRead = (Config::Rx::size - dmaChRW::counter()); nRead >= Config::Rx::idleMinSize) {
                            // f() -> bool, only reconfigure if parsing ok
                            if (!std::is_same_v<adapter, void> || // if adapter != void: allways call the following
                                f(dmaChRW::memoryAddress(), nRead) ||
                                (nRead == Config::Rx::size)
                                ) {
                                mcuUart->RQR = USART_RQR_RXFRQ; // clear fifo
                                mcuUart->CR3 &= ~USART_CR3_DMAR; // clear pending request
                                mcuUart->CR3 |= USART_CR3_DMAR;
                                dmaChRW::reConfigure([&]{
                                    if (dmaChRW::memoryAddress() == &mReadBuffer1[0]) {
                                        dmaChRW::memoryAddress(&mReadBuffer2[0]);
                                        mActiveReadBuffer = &mReadBuffer1[0];
                                        mCount1 = nRead;
                                        mActiveReadCount = &mCount1;
                                    }
                                    else {
                                        dmaChRW::memoryAddress(&mReadBuffer1[0]);
                                        mActiveReadBuffer = &mReadBuffer2[0];
                                        mCount2 = nRead;
                                        mActiveReadCount = &mCount2;
                                    }
                                    dmaChRW::size(Config::Rx::size);
                                });
                            }
                            mBufferHasData = true;
                        }
                    }
                }
            };
            template<bool Disable = true>
            static inline void baud(const uint32_t baud) {
                const auto [brr, presc] = calcBRR(baud);
                if constexpr(Disable) {
                    if constexpr(detail::getTxComplete_v<detail::getIsr_t<Config>>) {
                        mcuUart->CR1 &= (~USART_CR1_UE | ~USART_CR1_TCIE);
                    }
                    else {
                        mcuUart->CR1 &= ~USART_CR1_UE;
                    }
                }
                if constexpr ((N == 101) || (N == 102)) {
                    mcuUart->PRESC = presc;
                    mcuUart->BRR = brr;
                }
                else {
                    mcuUart->BRR = brr;
                }
                if constexpr(Disable) {
                    if constexpr(detail::getTxComplete_v<detail::getIsr_t<Config>>) {
                        mcuUart->CR1 |= (USART_CR1_UE | USART_CR1_TCIE);
                    }
                    else {
                        mcuUart->CR1 |= USART_CR1_UE;
                    }
                }
            }
            static inline void periodic()
                    requires (useDma && !std::is_same_v<adapter, void> && (Config::mode != Uarts::Mode::TxOnly)) {
                const auto [hasData, count] = Mcu::Arm::Atomic::access([]{
                    return std::pair{std::exchange(mBufferHasData, false), *mActiveReadCount};
                });
                if (hasData) {
                    for(uint16_t i = 0; i < count; ++i) {
                        adapter::process(mActiveReadBuffer[i]);
                    }
                }
            }
            static inline void periodic() requires(!useDma) {
                while(mcuUart->ISR & USART_ISR_TXE_TXFNF) {
                    value_t c;
                    if (mTxFifo.pop_front(c)) {
                        mcuUart->TDR = c;
                    }
                    else {
                        break;
                    }
                }
                if (mcuUart->ISR & USART_ISR_RXNE_RXFNE) {
                    // todo
                }
            }
            static inline void put(const value_t c) requires(!useDma) {
                mTxFifo.push_back(c);
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
            static inline tx_fifo_t mTxFifo;

            static inline rx_buffer_t mReadBuffer1;
            static inline rx_buffer_t mReadBuffer2;
            static inline storage_t* volatile mActiveReadBuffer = &mReadBuffer1[0];
            static inline volatile uint16_t mCount1 = 0;
            static inline volatile uint16_t mCount2 = 0;
            static inline bool volatile mBufferHasData = false;
            static inline volatile uint16_t* volatile mActiveReadCount = &mCount1;
            static inline tx_buffer_t mWriteBuffer1;
            static inline tx_buffer2_t mWriteBuffer2;
            static inline storage_t* volatile mActiveWriteBuffer = &mWriteBuffer1[0];
        };
    }
}
