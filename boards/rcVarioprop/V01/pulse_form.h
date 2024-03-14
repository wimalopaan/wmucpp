#pragma once

#include <cstring>

#include "meta.h"

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "clock.h"
#include "pwm.h"
#include "timer.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "gpio.h"
#include "tick.h"
#include "dac.h"

namespace Mcu::Stm {
    using namespace Units::literals;

    namespace Cppm {

        template<uint8_t N, typename DmaChannel, typename Dac, typename Clock, typename MCU>
        requires ((N >= 6) && (N <= 7))
        struct RollOnOff {
            using dac = Dac;
            using dmaChannel = DmaChannel;
            using value_type = uint16_t;
            using dac_value_type = dac::value_type;

            static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Timer<N, void, void, MCU>>::value);

            static inline constexpr uint16_t prescaler = (Clock::config::frequency.value / 1'000'000) - 1;

            static inline void init() {
                if constexpr(N == 6) {
                    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
                    dac::template dmaEnable<1>(7); // trg7
                }
                else if constexpr(N == 7) {
                    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
                    dac::template dmaEnable<1>(2); // trg2
                }
                else {
                    static_assert(false);
                }
                mcuTimer->PSC = prescaler;
                mcuTimer->ARR = 1; // 1MS/s

                update();

                dmaChannel::init();
                dmaChannel::template msize<value_type>();
                dmaChannel::template psize<dac_value_type>();

                dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_MINC;
                // dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_CIRC;
                dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_DIR;
                dmaChannel::mcuDmaChannel->CNDTR = samples;
                dmaChannel::mcuDmaChannel->CPAR = (uint32_t)&dac::mcuDac->DHR12R1;
                dmaChannel::mcuDmaChannel->CMAR = (uint32_t)&rollon[0];
                // dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_TCIE; // for test
                // dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_TEIE; // for test

                MODIFY_REG(dmaChannel::mcuDmaMux->CCR, DMAMUX_CxCR_DMAREQ_ID_Msk, dac::template dmamuxSrc<1>() << DMAMUX_CxCR_DMAREQ_ID_Pos);
                dmaChannel::mcuDmaMux->CCR |= DMAMUX_CxCR_EGE;

                dmaChannel::enable(true);

                // mcuTimer->DIER |= TIM_DIER_UDE;
                // mcuTimer->DIER |= TIM_DIER_UIE; // test
                // mcuTimer->EGR |= TIM_EGR_UG;
                // mcuTimer->CR1 |= TIM_CR1_ARPE;
                MODIFY_REG(mcuTimer->CR2, TIM_CR2_MMS_Msk, 0b010 << TIM_CR2_MMS_Pos); // update as trgo
                mcuTimer->CR1 |= TIM_CR1_CEN;
            }

            static inline void startRollon() {
                dmaChannel::enable(false);
                dmaChannel::mcuDmaChannel->CNDTR = samples;
                dmaChannel::mcuDmaChannel->CMAR = (uint32_t)&rollon[0];
                dmaChannel::enable(true);
            }
            static inline void startRolloff() {
                dmaChannel::enable(false);
                dmaChannel::mcuDmaChannel->CNDTR = samples;
                dmaChannel::mcuDmaChannel->CMAR = (uint32_t)&rolloff[0];
                dmaChannel::enable(true);
            }

            static inline void offset(const uint16_t o) {
                mOffset = o;
                update();
            }
            static inline void amplitude(const uint16_t a) {
                mAmplitude= a;
                update();
            }
        private:
            static inline void update() {
                for(uint16_t i = 0; i < samples; ++i) {
                    // rollon[i] = mOffset - mAmplitude * cos((i * std::numbers::pi) / samples);
                    // rolloff[i] = mOffset + mAmplitude * cos((i * std::numbers::pi) / samples);
                    rollon[i] = mOffset - mAmplitude * gauss(i);
                    rolloff[i] = mOffset + mAmplitude * gauss(i);
                }
            }
            // static constexpr uint16_t samples = 256; // 256µs (eine Flanke)
            static constexpr uint16_t samples = 200; // 200µs (eine Flanke)
            static inline std::array<volatile value_type, samples> rollon;
            static inline std::array<volatile value_type, samples> rolloff;
            static inline uint16_t mOffset{2048};
            static inline uint16_t mAmplitude{300};

            static inline constexpr float K = samples / 1.0f;
            static inline constexpr float tc0 = samples / (2.0 * K);
            static inline constexpr float c0 = exp(-tc0 * tc0);
            static inline float gauss(const uint16_t i) {
                if (i <= (samples / 2)) {
                    const float t = i / K;
                    return (exp(-t * t) - c0) / (1.0f - c0);
                }
                else {
                    const float t = (i - (samples - 1)) / K;
                    return (-exp(-t * t) + c0) / (1.0f - c0);
                }
            }
        };



    }
}


