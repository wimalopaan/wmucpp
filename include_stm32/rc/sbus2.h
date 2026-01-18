/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <cstdint>
#include <cstddef>
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <numbers>
#include <chrono>

#include "byte.h"
#include "etl/algorithm.h"
#include "etl/ranged.h"
#include "units.h"
#include "tick.h"

namespace RC {
    namespace Protokoll {
        namespace SBus2 {
            using namespace etl::literals;
            using namespace std::literals::chrono_literals;
            static inline constexpr uint32_t baud = 100'000;
            static inline constexpr std::byte start_byte = 0x0f_B;
            static inline constexpr std::byte end_byte = 0x00_B;

            static inline constexpr uint8_t flagsIndex = 23;
            static inline constexpr uint8_t endIndex = 24;

            static inline constexpr std::byte ch17 = 0x01_B;
            static inline constexpr std::byte ch18 = 0x02_B;
            static inline constexpr std::byte frameLost = 0x04_B;
            static inline constexpr std::byte failSafe = 0x08_B;

            inline static constexpr uint16_t sbus_min = 172;
            inline static constexpr uint16_t sbus_max = 1811;

            inline static constexpr uint16_t sbus_mid = (sbus_max + sbus_min) / 2;

            using value_type = etl::ranged<sbus_min, sbus_max>;
            using index_type = etl::ranged<0, 15>;

            namespace V2 {

            }

            namespace V1 {
                template<uint8_t UartNumber, typename DmaChannel, typename Clock, typename MCU = DefaultMcu>
                struct Master {
                    static inline /*constexpr */ USART_TypeDef* const mcuUart = reinterpret_cast<USART_TypeDef*>(Mcu::Stm::Address<Mcu::Stm::Uart<UartNumber, void, 0, void, Clock, MCU>>::value);

                    using dmaChannel = DmaChannel;

                    static inline void init() {
                        mData[0] = start_byte;
                        mData[flagsIndex] = 0_B;
                        mData[endIndex] = end_byte;

                        if constexpr (UartNumber == 1) {
                            RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
                            RCC->CCIPR |= 0x01 << RCC_CCIPR_USART1SEL_Pos;
                        }
                        else if constexpr (UartNumber == 2) {
                            RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
                            RCC->CCIPR |= 0x01 << RCC_CCIPR_USART2SEL_Pos;
                        }
                        else if constexpr (UartNumber == 3) {
                            RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
                            RCC->CCIPR |= 0x01 << RCC_CCIPR_USART3SEL_Pos;
                        }
                        else if constexpr (UartNumber == 101) {
                            RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
                            RCC->CCIPR |= 0x01 << RCC_CCIPR_LPUART1SEL_Pos;
                        }
                        else {
                            static_assert(false);
                        }
                        if constexpr (UartNumber == 101) {
                            mcuUart->PRESC = 0b0111; // 16
                            mcuUart->BRR = (16 * static_cast<Units::hertz>(Clock::config::f).value) / baud;
                        }
                        else {
                            mcuUart->PRESC = 0;
                            mcuUart->BRR = static_cast<Units::hertz>(Clock::config::f).value / baud;
                        }
                        mcuUart->CR1 |= USART_CR1_FIFOEN;
                        mcuUart->CR1 |= USART_CR1_RE;
                        mcuUart->CR1 |= USART_CR1_TE;
                        mcuUart->CR1 |= USART_CR1_PCE;
                        mcuUart->CR1 &= ~USART_CR1_PS;
                        mcuUart->CR1 |= USART_CR1_M0;

                        mcuUart->CR2 |= (USART_CR2_TXINV | USART_CR2_RXINV);

                        mcuUart->CR3 |= USART_CR3_HDSEL;
                        mcuUart->CR3 |= USART_CR3_DMAT;


                        mcuUart->CR1 |= USART_CR1_UE;


                        dmaChannel::init();
                        dmaChannel::template msize<std::byte>();
                        dmaChannel::template psize<std::byte>();

                        dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_MINC;
                        dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_CIRC;
                        dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_DIR;
                        dmaChannel::mcuDmaChannel->CNDTR = mData.size();
                        dmaChannel::mcuDmaChannel->CPAR = (uint32_t)(&mcuUart->TDR);
                        dmaChannel::mcuDmaChannel->CMAR = (uint32_t)&mData[0];
                        dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_TCIE;
                        dmaChannel::enable();

                        dmaChannel::mcuDmaMux->CCR = Mcu::Stm::Uarts::Properties<UartNumber>::dmamux_tx_src & DMAMUX_CxCR_DMAREQ_ID_Msk;
                        dmaChannel::mcuDmaMux->CCR |= DMAMUX_CxCR_EGE;
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
                    struct Interrupt {
                        static inline void dma() {
                            dmaChannel::clearTransferCompleteIF();
                        }
                        static inline void uart() {
                            if (mcuUart->ISR & USART_ISR_TC) {
                                transmissionComplete();
                            }
                        }
                    private:
                        static inline void transmissionComplete() {

                        }
                    };
                    template <typename C>
                    static inline void set(const C& v) {
                        // etl::copy(v, output);
                    }
                private:
                    static inline std::array<std::byte, 25> mData{};
                };

                template <typename Usart, typename Timer, typename dbg = void>
                struct Fsm {
                    using usart = Usart;
                    using pa = usart::pa_t;

                    static constexpr External::Tick<Timer> timeoutTicks{ 8ms };
                    // static_assert(timeoutTicks.value > 1);
                    //                std::integral_constant<uint8_t, timeoutTicks.value>::_;


                    inline static void init() {
                        // usart::template init<AVR::BaudRate<100000>, AVR::FullDuplex, true, 1>();
                        for (auto& o : output) {
                            o = (sbus_max + sbus_min) / 2;
                        }
                    }

                    static inline void set(const index_type& i, const value_type& v) {
                        output[i] = v;
                    }
                    template <uint8_t I>
                    static inline void set(const value_type& v) {
                        output[I] = v;
                    }

                    template <typename C>
                    static inline void set(const C& v) {
                        etl::copy(v, output);
                    }

                    static inline void switches(const std::byte s) {
                        static constexpr std::byte mask = ch17 | ch18;
                        mFlagsAndSwitches = (mFlagsAndSwitches & ~mask) | (s & mask);
                    }

                    static inline constexpr std::byte sbus_start = 0x0f_B;

                    inline static void periodic() {
                        usart::periodic();
                    }

                    inline static void start() {
                        mActive = true;
                        pa::active(false);
                        mState = State::SendFrame;
                    }
                    inline static void stop() {
                        mActive = false;
                    }

                    enum class State : uint8_t { SendFrame, WaitForSlot, ReceiveSlots };

                    inline static void ratePeriodic() { // 14ms
                        if (!mActive)
                            return;
                        const auto oldState = mState;
                        ++mStateTicks;
                        switch (mState) {
                        case State::SendFrame:
                            sendFrame();
                            mRequestIndex++;
                            if (mRequestIndex >= request.size()) {
                                mRequestIndex = 0;
                            }
                            mState = State::WaitForSlot;
                            break;
                        case State::WaitForSlot:
                            if (usart::isIdle() && usart::isTxQueueEmpty()) {
                                pa::active();
                                mState = State::ReceiveSlots;
                            }
                            break;
                        case State::ReceiveSlots:
                            mStateTicks.on(timeoutTicks, [] {
                                pa::active(false);
                                mState = State::SendFrame;
                                });
                            break;
                        }
                        if (mState != oldState) {
                            mStateTicks.reset();
                        }
                    }
                private:
                    static inline void sendFrame() {
                        usart::put(sbus_start);
                        usart::put((std::byte)(output[0] & 0x07FF));
                        usart::put((std::byte)((output[0] & 0x07FF) >> 8 | (output[1] & 0x07FF) << 3));
                        usart::put((std::byte)((output[1] & 0x07FF) >> 5 | (output[2] & 0x07FF) << 6));
                        usart::put((std::byte)((output[2] & 0x07FF) >> 2));
                        usart::put((std::byte)((output[2] & 0x07FF) >> 10 | (output[3] & 0x07FF) << 1));
                        usart::put((std::byte)((output[3] & 0x07FF) >> 7 | (output[4] & 0x07FF) << 4));
                        usart::put((std::byte)((output[4] & 0x07FF) >> 4 | (output[5] & 0x07FF) << 7));
                        usart::put((std::byte)((output[5] & 0x07FF) >> 1));
                        usart::put((std::byte)((output[5] & 0x07FF) >> 9 | (output[6] & 0x07FF) << 2));
                        usart::put((std::byte)((output[6] & 0x07FF) >> 6 | (output[7] & 0x07FF) << 5));
                        usart::put((std::byte)((output[7] & 0x07FF) >> 3));
                        usart::put((std::byte)((output[8] & 0x07FF)));
                        usart::put((std::byte)((output[8] & 0x07FF) >> 8 | (output[9] & 0x07FF) << 3));
                        usart::put((std::byte)((output[9] & 0x07FF) >> 5 | (output[10] & 0x07FF) << 6));
                        usart::put((std::byte)((output[10] & 0x07FF) >> 2));
                        usart::put((std::byte)((output[10] & 0x07FF) >> 10 | (output[11] & 0x07FF) << 1));
                        usart::put((std::byte)((output[11] & 0x07FF) >> 7 | (output[12] & 0x07FF) << 4));
                        usart::put((std::byte)((output[12] & 0x07FF) >> 4 | (output[13] & 0x07FF) << 7));
                        usart::put((std::byte)((output[13] & 0x07FF) >> 1));
                        usart::put((std::byte)((output[13] & 0x07FF) >> 9 | (output[14] & 0x07FF) << 2));
                        usart::put((std::byte)((output[14] & 0x07FF) >> 6 | (output[15] & 0x07FF) << 5));
                        usart::put((std::byte)((output[15] & 0x07FF) >> 3));
                        usart::put(mFlagsAndSwitches); // Flags byte
                        usart::put(request[mRequestIndex]);            // Telem-Request
                    }
                    static inline constexpr std::array<std::byte, 4> request{ 0x04_B, 0x14_B, 0x24_B, 0x34_B };
                    static inline uint8_t mRequestIndex{ 0 };
                    static inline State mState{ State::SendFrame };
                    static inline bool mActive{ true };
                    static inline std::byte mFlagsAndSwitches{};
                    static inline std::array<uint16_t, 16> output;
                    static inline External::Tick<Timer> mStateTicks{};
                };
                template<uint8_t N, typename Callback>
                struct Adapter {
                    enum class State : uint8_t { InActive, Slots };
                    static inline void reset() {
                        mIndex = 0;
                        mSlot = 0;
                        mState = State::InActive;
                    }
                    static inline void active(const bool active = true) {
                        if (active) {
                            mIndex = 0;
                            mSlot = 0;
                            mState = State::Slots;
                        }
                        else {
                            mState = State::InActive;
                        }
                    }
                    static inline void process(const std::byte b) {
                        switch (mState) {
                        case State::InActive:
                            break;
                        case State::Slots:
                            mSlots[mSlot][mIndex++] = b;
                            if (mIndex >= 3) {
                                mIndex = 0;
                                mSlot++;
                                if (mSlot >= mSlots.size()) {
                                    mSlot = 0;
                                    mIndex = 0;
                                    mState = State::InActive;
                                    Callback::update();
                                }
                            }
                            break;
                        }
                    }
                    //                private:
                    static inline uint8_t mIndex{ 0 };
                    static inline uint8_t mSlot{ 0 };
                    static inline std::array<std::array<std::byte, 3>, 5> mSlots;
                    static inline State mState{ State::InActive };
                };
            }

        }
    }
}
