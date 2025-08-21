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
#include "dma_dual_2.h"
#include "exti.h"
#include "output.h"
#include "timer.h"

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
    static inline constexpr bool RxTxLinesDifferent = false; // HalfDuplex-Only
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
        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct Uart;

        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        using LpUart = Uart<N + 100, Config, MCU>;

        // software UART: read-only
        // uses EXTI
        template<typename Config, typename MCU>
        struct Uart<0, Config, MCU> {
            using clock = Config::Clock;
            using debug = Config::debug;
            using pin = Config::pin;
            using gpio = pin::gpio_t;
            using port = gpio::port_t;
            using tp = Config::tp;
            static inline constexpr uint8_t N = pin::number;
            static inline constexpr uint8_t exti_n = pin::number / 4;
            static inline constexpr uint8_t exti_s = pin::number % 4;

            static inline constexpr uint8_t map = Mcu::Stm::ExtI::Map<port>::value;

            static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Timer<Config::timerN>>::value);
            static inline constexpr uint16_t bitPeriod = (clock::config::frequency.value / Config::baudrate);
            // using std::integral_constant<uint16_t, bitPeriod>::_;

            static inline constexpr uint8_t numberOfBits = []{
                if constexpr(detail::getParity_v<Config> == Uarts::Parity::Even) {
                    return 10;
                }
                else if constexpr(detail::getParity_v<Config> == Uarts::Parity::Odd) {
                    return 10;
                }
                else {
                    return 9;
                }
            }();
            // using std::integral_constant<uint16_t, numberOfBits>::_;

            static inline void init() {
                IO::outl<debug>("# Uart 0 (Soft): ", N, ", ", exti_n, ", ", exti_s, ", ", map);
                Mcu::Stm::Timers::powerUp<Config::timerN>();
                if constexpr(exti_n < 4) {
                    EXTI->EXTICR[exti_n] = map << (exti_s * 8);
                }
                else {
                    static_assert(false);
                }
                EXTI->RTSR1 |= (0x01 << N);
                EXTI->IMR1  |= (0x01 << N);
                mcuTimer->PSC = 0; // prescaler == 1
                mcuTimer->ARR = bitPeriod;
                mcuTimer->DIER |= TIM_DIER_UIE;
                mcuTimer->EGR |= TIM_EGR_UG;
                mcuTimer->CR1 |= TIM_CR1_URS;
                mByteCount1 = 0;
                mByteCount2 = 0;
                mState = State::Idle;
            }
            static inline void reset() {
                Mcu::Stm::Timers::reset<Config::timerN>();
                // reset EXTI
                EXTI->RTSR1 &= ~(0x01 << N);
                EXTI->IMR1  &= ~(0x01 << N);
            }
            template<bool Enable>
            static inline void rxEnable() {}
            static inline auto readBuffer(const auto f) {
                if (mActiveDataPtr == &mData1[0]) {
                    f(mData2);
                }
                else {
                    f(mData1);
                }
            }
            static inline void onParityGood(auto f) {
                if (!mParityErrorSave) {
                    f();
                }
                mParityErrorSave = false;
            }

            enum class State : uint8_t {Idle, Receiving};

            static inline auto ratePeriodic() {
                if (mState == State::Receiving) {
                    mRateCount = mRateCount + 1;
                    if (mRateCount >= 2) {
                        if constexpr(!std::is_same_v<tp, void>) {
                            tp::set();
                        }
                        const auto d_ptr = mActiveDataPtr;
                        const auto d_s = *mActiveByteCount;
                        if (mActiveDataPtr == &mData1[0]) {
                            mActiveDataPtr = &mData2[0];
                            mActiveByteCount = &mByteCount2;
                        }
                        else {
                            mActiveDataPtr = &mData1[0];
                            mActiveByteCount = &mByteCount1;
                        }
                        *mActiveByteCount = 0;
                        mParityErrorSave = mParityError;
                        mParityError = false;
                        Config::callback::idle(d_ptr, d_s);
                        mState = State::Idle;
                        if constexpr(!std::is_same_v<tp, void>) {
                            tp::reset();
                        }
                    }
                }
            }
            struct Isr {
                static inline void edge() {
                    mState = State::Receiving;
                    EXTI->RPR1 = (0x01 << N);
                    mRateCount = 0;
                    mBitCount = 0;
                    mFrame = 0;
                    mParity = false;
                    mcuTimer->ARR = ((bitPeriod / 2) * 6) / 10;
                    mcuTimer->CNT = 0;
                    mcuTimer->CR1 |= TIM_CR1_CEN;
                    EXTI->IMR1 &= ~(0x01 << N);
                }
                static inline void period() {
                    if constexpr(!std::is_same_v<tp, void>) {
                        tp::set();
                    }
                    const bool bit = Config::invert ? !pin::read() : pin::read();
                    mcuTimer->SR = ~TIM_SR_UIF;
                    mcuTimer->ARR = bitPeriod - 1;
                    // lsb first
                    mFrame = (mFrame >> 1);
                    if (bit) {
                        mFrame = mFrame | (0b1 << (numberOfBits - 1));
                        mParity = !mParity;
                    }
                    mBitCount = mBitCount + 1;
                    if (mBitCount >= numberOfBits) {
                        mcuTimer->CR1 &= ~TIM_CR1_CEN;
                        EXTI->IMR1 = (0x01 << N);
                        if constexpr(detail::getParity_v<Config> == Uarts::Parity::Even) {
                            mParityError |= (mParity != (mFrame & 0b01));
                        }
                        else if constexpr(detail::getParity_v<Config> == Uarts::Parity::Odd) {
                            mParityError |= (mParity == (mFrame & 0b01));
                        }
                        if (*mActiveByteCount < mData1.size()) {
                            mActiveDataPtr[*mActiveByteCount] = (mFrame >> 1) & 0xff; // omit parity and start bit
                            *mActiveByteCount = *mActiveByteCount + 1;
                        }
                    }
                    if constexpr(!std::is_same_v<tp, void>) {
                        tp::reset();
                    }
                }
            };
            private:
            static inline std::array<volatile uint8_t, Config::Rx::size> mData1;
            static inline std::array<volatile uint8_t, Config::Rx::size> mData2;
            static inline volatile uint8_t* mActiveDataPtr = &mData1[0];
            static inline volatile uint8_t mByteCount1 = 0;
            static inline volatile uint8_t mByteCount2 = 0;
            static inline volatile uint8_t* mActiveByteCount = &mByteCount1;
            static inline volatile State mState = State::Idle;
            static inline volatile bool mParityErrorSave = false;
            static inline volatile bool mParityError = false;
            static inline volatile bool mParity = false;
            static inline volatile uint16_t mFrame = 0;
            static inline volatile uint8_t mBitCount = 0;
            static inline volatile uint16_t mRateCount = 0;
        };

        template<uint8_t N, typename Config, typename MCU>
        requires (
                ((N >= 1) && (N <= 2) && std::is_same_v<Stm32G030, MCU>) ||
                ((N >= 1) && (N <= 2) && std::is_same_v<Stm32G031, MCU>) ||
                ((N >= 1) && (N <= 2) && std::is_same_v<Stm32G051, MCU>) ||
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
            using dmaChRW = Mcu::Stm::Dma::V2::DualChannel<rwDmaComponent_t, rxDmaComponent_t, txDmaComponent_t, dmaChConfig>;

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
                        if constexpr(!detail::getRxTxLinesDifferent_v<Config>) {
                            cr3 |= USART_CR3_HDSEL;
                        }
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
                // mcuUart->ICR = USART_ICR_TCCF;
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
                if (n > 0) {
                    startSend(n);
                }
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
                        if constexpr(requires(){{f()} -> std::convertible_to<bool>;}) {
                            const bool e = f();
                            if constexpr(Config::mode == Uarts::Mode::HalfDuplex) {
                                if (e) {
                                    rxEnable<true>();
                                }
                            }
                        }
                        else {
                            f();
                            if constexpr(Config::mode == Uarts::Mode::HalfDuplex) {
                                rxEnable<true>();
                            }
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
            static inline void halfDuplex(const bool on = true)
                    requires(Config::mode == Uarts::Mode::FullDuplex)
            {
                if constexpr(Disable) {
                    if constexpr(detail::getTxComplete_v<detail::getIsr_t<Config>>) {
                        mcuUart->CR1 &= ~(USART_CR1_UE | USART_CR1_TCIE);
                    }
                    else {
                        mcuUart->CR1 &= ~USART_CR1_UE;
                    }
                }
                if (on) {
                    mcuUart->CR3 |= USART_CR3_HDSEL;
                }
                else {
                    mcuUart->CR3 &= ~USART_CR3_HDSEL;
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
            template<bool Inv>
            static inline void invert() {
                if constexpr(detail::getTxComplete_v<detail::getIsr_t<Config>>) {
                    mcuUart->CR1 &= ~(USART_CR1_UE | USART_CR1_TCIE);
                }
                else {
                    mcuUart->CR1 &= ~USART_CR1_UE;
                }
                if constexpr(Inv) {
                    mcuUart->CR2 |= (USART_CR2_TXINV | USART_CR2_RXINV);
                }
                else {
                    mcuUart->CR2 &= ~(USART_CR2_TXINV | USART_CR2_RXINV);
                }
                if constexpr(detail::getTxComplete_v<detail::getIsr_t<Config>>) {
                    mcuUart->CR1 |= (USART_CR1_UE | USART_CR1_TCIE);
                }
                else {
                    mcuUart->CR1 |= USART_CR1_UE;
                }
            }
            template<bool Disable = true>
            static inline void baud(const uint32_t baud) {
                const auto [brr, presc] = calcBRR(baud);
                if constexpr(Disable) {
                    if constexpr(detail::getTxComplete_v<detail::getIsr_t<Config>>) {
                        mcuUart->CR1 &= ~(USART_CR1_UE | USART_CR1_TCIE);
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
                const auto [hasData, count] = Mcu::Arm::Atomic::access([] static {
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
