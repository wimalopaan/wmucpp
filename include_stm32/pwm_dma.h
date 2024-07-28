#pragma once

#include <cstdint>
#include <chrono>
#include <type_traits>
#include <output.h>
#include <mcu/alternate.h>

#include "tick.h"

namespace External {
    using namespace std::literals::chrono_literals;

    struct Set;
    struct Reset;

    template<typename Pin, typename Kind, typename DmaChannel, auto Source, typename MCU = DefaultMcu>
    struct PinDma {
        using dmaChannel = DmaChannel;
        static inline void init() {
            Pin::template dir<Mcu::Output>();
            dmaChannel::init();
            dmaChannel::template msize<uint32_t>();
            dmaChannel::template psize<uint32_t>();

            dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_CIRC;
            dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_DIR;
            dmaChannel::mcuDmaChannel->CNDTR = 1;
            dmaChannel::mcuDmaChannel->CPAR = (uint32_t)&Pin::mcuGpio->BSRR;

            // if constexpr(dmaChannel::number == 1) {
            //     dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_TCIE; // for test
            //     dmaChannel::mcuDmaChannel->CCR |= DMA_CCR_TEIE; // for test
            // }

            MODIFY_REG(dmaChannel::mcuDmaMux->CCR, DMAMUX_CxCR_DMAREQ_ID_Msk, Source << DMAMUX_CxCR_DMAREQ_ID_Pos);
            dmaChannel::mcuDmaMux->CCR |= DMAMUX_CxCR_EGE;

            if constexpr(std::is_same_v<Kind, Set>) {
                dmaChannel::mcuDmaChannel->CMAR = (uint32_t)&setMask;
            }
            else if constexpr(std::is_same_v<Kind, Reset>) {
                dmaChannel::mcuDmaChannel->CMAR = (uint32_t)&resetMask;
            }
            else {
                static_assert(false);
            }

            dmaChannel::enable(true);
        }
        private:
        static inline volatile uint32_t setMask = (0x01UL << Pin::number);
        static inline volatile uint32_t resetMask = (0x01UL << (Pin::number + 16));
    };
}
