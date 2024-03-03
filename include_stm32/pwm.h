#pragma once

#include <type_traits>
#include <concepts>
#include <array>

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/alternate.h"
#include "units.h"
#include "concepts.h"
#include "meta.h"

#include "timer.h"
#include "dma.h"

namespace Mcu::Stm {
    using namespace Units::literals;

#ifdef USE_MCU_STM_V2
    inline
#endif
        namespace V2 {
        namespace Pwm {
            // Cyclic DMA        
            template<uint8_t TimerNumber, uint8_t MaxLength, typename DmaChannelList, typename Clock, typename MCU = DefaultMcu>
            struct Sequence;

            template<uint8_t TimerNumber, uint8_t MaxLength, typename... DmaChannels, typename Clock, typename MCU>
            struct Sequence<TimerNumber, MaxLength, Meta::List<DmaChannels...>, Clock, MCU> {
                static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Timer<TimerNumber, void, void, MCU>>::value);

                using value_type = uint16_t;
                using timer_value_type = Mcu::Stm::Timers::Properties<TimerNumber>::value_type;

                static inline constexpr uint16_t onems = 1640;
                static inline constexpr uint16_t off = 1.5f * onems;
                static inline constexpr uint16_t s1 = 1.1f * onems;
                static inline constexpr uint16_t s2 = 1.9f * onems;
                static inline constexpr uint16_t sync = 2.2f * onems;
                static inline constexpr uint16_t period = onems * 20;
                static inline constexpr uint16_t prescaler = (Clock::config::frequency.value / period) / 50;

                using dmaChannelList = Meta::List<DmaChannels...>;
                using dmaControllers = Meta::List<typename DmaChannels::controller...>;
                static_assert(Meta::all_same_front_v<dmaControllers>);
                using dma = Meta::front<dmaControllers>;

                static inline constexpr uint8_t numberOfChannels = Meta::size_v<dmaChannelList>;

                template<typename DmaChannel, auto N>
                struct Single {
                    using dmaChannel = DmaChannel;
                    static inline void init() {
                        dmaChannel::init();
                        dmaChannel::template msize<value_type>();
                        dmaChannel::template psize<timer_value_type>();

                        dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_MINC;
                        dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_CIRC;
                        dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_DIR;
                        dmaChannel::mcuDmaChannel->CNDTR = mSequenceLength + mSyncPulses;
                        dmaChannel::mcuDmaChannel->CPAR = (uint32_t)(&mcuTimer->CCR1 + N);
                        dmaChannel::mcuDmaChannel->CMAR = (uint32_t)&mValues[0];
                        if constexpr(N == 0) {
                            dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_TCIE; // for test
                        }
                        dmaChannel::enable();

                        dmaChannel::mcuDmaMux->CCR = Mcu::Stm::Timers::Properties<TimerNumber>::dmamux_src[N] & DMAMUX_CxCR_DMAREQ_ID_Msk;
                        dmaChannel::mcuDmaMux->CCR |= DMAMUX_CxCR_EGE;
                        *(&mcuTimer->CCR1 + N) = mValues[0];

                        sequence(MaxLength - 2, 2);
                    }
                    static inline void sequence(const uint8_t pulses, const uint8_t syncPulses) {
                        if ((pulses + syncPulses) <= MaxLength) {
                            for (auto& v : mValues) {
                                v = off;
                            }
                            mSyncPulses = syncPulses;
                            mSequenceLength = pulses;
                            for (uint8_t i = 0; i < mSyncPulses; ++i) {
                                mValues[mSequenceLength + i] = sync;
                            }
                            dmaChannel::mcuDmaChannel->CCR &= ~DMA_CCR_EN;
                            dmaChannel::mcuDmaChannel->CNDTR = mSequenceLength + mSyncPulses;
                            dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_EN;
                        }
                    }
                    static inline void set(const uint8_t s, const bool on = true) {
                        const uint8_t n = s / 2;
                        const uint8_t r = s % 2;
                        if (n < mSequenceLength) {
                            if (on) {
                                mValues[n] = (r == 0) ? s1 : s2;
                            }
                            else {
                                mValues[n] = off;
                            }
                        }
                    }
                private:
                    static inline uint8_t mSyncPulses{ 2 };
                    static inline uint8_t mSequenceLength = MaxLength - mSyncPulses;
                    static inline std::array<value_type, MaxLength>  mValues{};
                };

                using pwms = Meta::transformN<Single, dmaChannelList>;

                static inline void init() {
                    if constexpr (TimerNumber == 1) {
                        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
                    }
                    else if constexpr (TimerNumber == 2) {
                        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
                    }
                    else if constexpr (TimerNumber == 3) {
                        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
                    }
                    else if constexpr (TimerNumber == 4) {
                        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
                    }
                    else {
                        static_assert(false);
                    }
                    mcuTimer->PSC = prescaler;
                    mcuTimer->ARR = period;

                    // sequence(MaxLength - 2, 2);

                    Meta::visit<pwms>([]<typename S>(Meta::Wrapper<S>) {
                        S::init();
                    });

                    // dmaChannel::init();
                    // dmaChannel::template msize<value_type>();
                    // dmaChannel::template psize<timer_value_type>();

                    // dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_MINC;
                    // dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_CIRC;
                    // dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_DIR;
                    // dmaChannel::mcuDmaChannel->CNDTR = mSequenceLength + mSyncPulses;
                    // dmaChannel::mcuDmaChannel->CPAR = (uint32_t)&mcuTimer->CCR1;
                    // dmaChannel::mcuDmaChannel->CMAR = (uint32_t)&mValues[0];
                    // dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_TCIE; // for test
                    // dmaChanndmael::enable();

                    // dmaChannel::mcuDmaMux->CCR = Mcu::Stm::Timers::Properties<TimerNumber>::dmamux_src[0] & DMAMUX_CxCR_DMAREQ_ID_Msk; 
                    // dmaChannel::mcuDmaMux->CCR |= DMAMUX_CxCR_EGE;

                    // Enable Timer, DMA
                    if constexpr (numberOfChannels > 0) {
                        mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC1M_Pos); // pwm1
                        mcuTimer->CCMR1 |= TIM_CCMR1_OC1PE;
                        mcuTimer->CCER |= TIM_CCER_CC1E;
                        mcuTimer->DIER |= TIM_DIER_CC1DE;
                    }
                    if constexpr (numberOfChannels > 1) {
                        mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC2M_Pos);
                        mcuTimer->CCMR1 |= TIM_CCMR1_OC2PE;
                        mcuTimer->CCER |= TIM_CCER_CC2E;
                        mcuTimer->DIER |= TIM_DIER_CC2DE;
                    }
                    if constexpr (numberOfChannels > 2) {
                        mcuTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC3M_Pos);
                        mcuTimer->CCMR2 |= TIM_CCMR2_OC3PE;
                        mcuTimer->CCER |= TIM_CCER_CC3E;
                        mcuTimer->DIER |= TIM_DIER_CC3DE;
                    }
                    if constexpr (numberOfChannels > 3) {
                        mcuTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC4M_Pos);
                        mcuTimer->CCMR2 |= TIM_CCMR2_OC4PE;
                        mcuTimer->CCER |= TIM_CCER_CC4E;
                        mcuTimer->DIER |= TIM_DIER_CC4DE;
                    }
                    mcuTimer->EGR |= TIM_EGR_UG;
                    mcuTimer->CR1 |= TIM_CR1_ARPE;
                    mcuTimer->CR1 |= TIM_CR1_CEN;
                }
                // static inline void sequence(const uint8_t pulses, const uint8_t syncPulses) {
                //     if ((pulses + syncPulses) <= MaxLength) {
                //         for(auto& v : mValues) {
                //             v = off;
                //         }
                //         mSyncPulses = syncPulses;
                //         mSequenceLength = pulses;
                //         for(uint8_t i = 0; i < mSyncPulses; ++i) {
                //             mValues[mSequenceLength + i] = sync;
                //         }
                //         DMA1_Channel1->CCR &= ~DMA_CCR_EN;
                //         DMA1_Channel1->CNDTR = mSequenceLength + mSyncPulses;
                //         DMA1_Channel1->CCR |= DMA_CCR_EN;                        
                //     }
                // }
                // static inline void set(const uint8_t s, const bool on = true) {
                //     const uint8_t n = s / 2;
                //     const uint8_t r = s % 2;
                //     if (n < mSequenceLength) {
                //         if (on) {
                //             mValues[n] = (r == 0) ? s1 : s2;
                //         }
                //         else {
                //             mValues[n] = off;
                //         }
                //     }
                // }
            private:
                // static inline uint8_t mSyncPulses{2};
                // static inline uint8_t mSequenceLength = MaxLength - mSyncPulses;
                // static inline std::array<value_type, MaxLength>  mValues{};
            };

            template<uint8_t TimerNumber, typename Clock, typename MCU = DefaultMcu>
            struct Servo {
                static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Timer<TimerNumber, void, void, MCU>>::value);

                static inline constexpr uint16_t onems = 1640;
                static inline constexpr uint16_t mid = onems + onems / 2;
                static inline constexpr uint16_t period = onems * 20;
                static inline constexpr uint16_t prescaler = (Clock::config::frequency.value / period) / 50;

                static inline void init() {
                    if constexpr (TimerNumber == 1) {
                        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
                    }
                    else if constexpr (TimerNumber == 2) {
                        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
                    }
                    else if constexpr (TimerNumber == 3) {
                        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
                    }
                    else if constexpr (TimerNumber == 4) {
                        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
                    }
                    else {
                        static_assert(false);
                    }
                    mcuTimer->PSC = prescaler;
                    mcuTimer->ARR = period;
                    mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC1M_Pos); // pwm1
                    mcuTimer->CCMR1 |= TIM_CCMR1_OC1PE;
                    mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC2M_Pos); // pwm2
                    mcuTimer->CCMR1 |= TIM_CCMR1_OC2PE;
                    mcuTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC3M_Pos); // pwm3
                    mcuTimer->CCMR2 |= TIM_CCMR1_OC1PE;
                    mcuTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC4M_Pos); // pwm4
                    mcuTimer->CCMR2 |= TIM_CCMR1_OC2PE;
                    mcuTimer->CCER |= TIM_CCER_CC1E;
                    mcuTimer->CCER |= TIM_CCER_CC2E;
                    mcuTimer->CCER |= TIM_CCER_CC3E;
                    mcuTimer->CCER |= TIM_CCER_CC4E;
                    mcuTimer->CCR1 = mid;
                    mcuTimer->CCR2 = mid;
                    mcuTimer->CCR3 = mid;
                    mcuTimer->CCR4 = mid;
                    mcuTimer->EGR |= TIM_EGR_UG;
                    mcuTimer->CR1 |= TIM_CR1_ARPE;
                    mcuTimer->CR1 |= TIM_CR1_CEN;
                }
                static inline void set(const uint8_t channel, uint16_t sbus) {
                    const uint16_t t = sbus - 172 + onems;
                    switch (channel) {
                    case 0:
                        mcuTimer->CCR1 = t;
                        break;
                    case 1:
                        mcuTimer->CCR2 = t;
                        break;
                    case 2:
                        mcuTimer->CCR3 = t;
                        break;
                    case 3:
                        mcuTimer->CCR4 = t;
                        break;
                    default:
                        break;
                    }
                }
            };
        }
    }
}
