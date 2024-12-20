#pragma once

#include <type_traits>
#include <concepts>

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "units.h"
#include "concepts.h"
#include "components.h"
#include "dmas.h"
#include "dma_controller.h"

namespace Mcu::Stm {
    namespace Dma {

        namespace V2 {
            template<uint8_t N, typename Config, typename MCU = DefaultMcu>
            struct Channel {
                static_assert((N >= 1) && (N <= 8));
                using controller = Config::controller;
                static inline constexpr uint8_t number = N;
                using contr_comp_t = controller::component_t;
                using component_t = Mcu::Components::DmaChannel<contr_comp_t, N>;
                using value_t = Config::value_t;

                static inline /*constexpr */ DMA_Channel_TypeDef* const mcuDmaChannel = reinterpret_cast<DMA_Channel_TypeDef*>(Mcu::Stm::Address<component_t>::value);
                static inline /*constexpr */ DMAMUX_Channel_TypeDef* const mcuDmaMux = reinterpret_cast<DMAMUX_Channel_TypeDef*>(Mcu::Stm::Address<component_t>::mux);

    #ifdef STM32G4
                static inline void init() {
                    RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
                }
    #endif
    #ifdef STM32G0
                static inline void init() {
                    mcuDmaChannel->CCR = []{
                        uint32_t ccr = 0;
                        if constexpr(sizeof(value_t) == 1) {
                            ccr |= (0b00 << DMA_CCR_MSIZE_Pos);
                            ccr |= (0b00 << DMA_CCR_PSIZE_Pos);
                        }
                        else if constexpr(sizeof(value_t) == 2) {
                            ccr |= (0b01 << DMA_CCR_MSIZE_Pos);
                            ccr |= (0b01 << DMA_CCR_PSIZE_Pos);
                        }
                        else if constexpr(sizeof(value_t) == 4) {
                            ccr |= (0b10 << DMA_CCR_MSIZE_Pos);
                            ccr |= (0b10 << DMA_CCR_PSIZE_Pos);
                        }
                        else {
                            static_assert(false);
                        }
                        if constexpr(Config::memoryIncrement) {
                            ccr |= DMA_CCR_MINC;
                        }
                        return ccr;
                    }();
                }
                static inline void reset() {
                    mcuDmaChannel->CCR = 0;
                }
    #endif
                static inline void memoryAddress(volatile value_t* adr) {
                    mcuDmaChannel->CMAR = (uint32_t)adr;
                }
                static inline volatile value_t* memoryAddress() {
                    return (volatile value_t*)mcuDmaChannel->CMAR;
                }
                static inline uint32_t counter() {
                    return mcuDmaChannel->CNDTR;
                }
                static inline void size(const uint32_t s) {
                    mcuDmaChannel->CNDTR = s;
                }
                static inline void clearMux() {
                    MODIFY_REG(mcuDmaMux->CCR, DMAMUX_CxCR_DMAREQ_ID_Msk, 0);
                }
                static inline void startRead(const size_t size, const uint32_t pAdr, volatile value_t* mAdr, uint8_t mux) {
                    mcuDmaChannel->CCR &= ~DMA_CCR_EN;
                    MODIFY_REG(mcuDmaMux->CCR, DMAMUX_CxCR_DMAREQ_ID_Msk, mux << DMAMUX_CxCR_DMAREQ_ID_Pos);
                    mcuDmaChannel->CNDTR = size;
                    mcuDmaChannel->CPAR = pAdr;
                    mcuDmaChannel->CMAR = (uint32_t)mAdr;
                    mcuDmaChannel->CCR = []{
                        uint32_t ccr = mcuDmaChannel->CCR;
                        ccr &= ~DMA_CCR_DIR;
                        ccr |= DMA_CCR_EN;
                        return ccr;
                    }();
                }
                static inline void startWrite(const size_t size, const uint32_t pAdr, volatile value_t* mAdr, uint8_t mux) {
                    mcuDmaChannel->CCR &= ~DMA_CCR_EN;
                    MODIFY_REG(mcuDmaMux->CCR, DMAMUX_CxCR_DMAREQ_ID_Msk, mux << DMAMUX_CxCR_DMAREQ_ID_Pos);
                    clearAllFlags();
                    mcuDmaChannel->CNDTR = size;
                    mcuDmaChannel->CPAR = pAdr;
                    mcuDmaChannel->CMAR = (uint32_t)mAdr;
                    mcuDmaChannel->CCR = []{
                        uint32_t ccr = mcuDmaChannel->CCR;
                        ccr |= DMA_CCR_DIR;
                        ccr |= DMA_CCR_EN;
                        return ccr;
                    }();
                }

                static inline void enable(const bool on = true) {
                    if (on) {
                        mcuDmaChannel->CCR |= DMA_CCR_EN;
                    }
                    else {
                        mcuDmaChannel->CCR &= ~DMA_CCR_EN;
                    }
                }

                static inline void reConfigure(const auto f) {
                    enable(false);
                    f();
                    enable(true);
                }
                static inline void clearAllFlags() {
                    controller::mcuDma->IFCR = 0b1111UL << (4 * (N - 1));
                }
                static inline void clearTransferCompleteIF() {
                    controller::mcuDma->IFCR = 0x1UL << (4 * (N - 1) + 1);
                }
                static inline bool transferError() {
                    if (controller::mcuDma->ISR & (0x1UL << (4 * (N - 1) + 3))) {
                        return true;
                    }
                    return false;
                }
                // static inline void onTransferComplete(auto f) {
                //     if (controller::mcuDma->ISR & (0x1UL << (4 * (N - 1) + 1))) {
                //         clearTransferCompleteIF();
                //         f();
                //     }
                // }
            };
        }


    }
}
