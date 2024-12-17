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
        using namespace Units::literals;

        inline
        namespace V1 {
            template<typename Controller, uint8_t N = 0, typename MCU = DefaultMcu> struct Channel;

            template<uint8_t CNumber, uint8_t N, typename MCU>
            struct Channel<Controller<CNumber, MCU>, N, MCU> {
                static_assert((N >= 1) && (N <= 8));
                using controller = Controller<CNumber, MCU>;
                static inline constexpr uint8_t number = N;

                static inline /*constexpr */ DMA_Channel_TypeDef* const mcuDmaChannel = reinterpret_cast<DMA_Channel_TypeDef*>(Mcu::Stm::Address<Channel<controller, N, MCU>>::value);
                static inline /*constexpr */ DMAMUX_Channel_TypeDef* const mcuDmaMux = reinterpret_cast<DMAMUX_Channel_TypeDef*>(Mcu::Stm::Address<Channel<controller, N, MCU>>::mux);

    #ifdef STM32G4
                static inline void init() {
                    controller::init();
                    RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
                }
    #endif
    #ifdef STM32G0
                static inline void init() {
                    controller::init();
                }
    #endif
                template<typename T>
                static inline void msize() {
                    if constexpr(sizeof(T) == 1) {
                        MODIFY_REG(mcuDmaChannel->CCR, DMA_CCR_MSIZE_Msk, 0b00 << DMA_CCR_MSIZE_Pos);
                    }
                    else if constexpr(sizeof(T) == 2) {
                        MODIFY_REG(mcuDmaChannel->CCR, DMA_CCR_MSIZE_Msk, 0b01 << DMA_CCR_MSIZE_Pos);
                    }
                    else if constexpr(sizeof(T) == 4) {
                        MODIFY_REG(mcuDmaChannel->CCR, DMA_CCR_MSIZE_Msk, 0b10 << DMA_CCR_MSIZE_Pos);
                    }
                    else {
                        static_assert(false);
                    }
                }
                template<typename T>
                static inline void psize() {
                    if constexpr(sizeof(T) == 1) {
                        MODIFY_REG(mcuDmaChannel->CCR, DMA_CCR_PSIZE_Msk, 0b00 << DMA_CCR_PSIZE_Pos);
                    }
                    else if constexpr(sizeof(T) == 2) {
                        MODIFY_REG(mcuDmaChannel->CCR, DMA_CCR_PSIZE_Msk, 0b01 << DMA_CCR_PSIZE_Pos);
                    }
                    else if constexpr(sizeof(T) == 4) {
                        MODIFY_REG(mcuDmaChannel->CCR, DMA_CCR_PSIZE_Msk, 0b10 << DMA_CCR_PSIZE_Pos);
                    }
                    else {
                        static_assert(false);
                    }
                }
                static inline void enable(const bool on = true) {
                    if (on) {
                        mcuDmaChannel->CCR |= DMA_CCR_EN;
                    }
                    else {
                        mcuDmaChannel->CCR &= ~DMA_CCR_EN;
                    }
                }
                // deprecate
                template<bool Enable = true>
                static inline void enableTCIsr() {
                    enable(false);
                    if constexpr(Enable) {
                        mcuDmaChannel->CCR |= DMA_CCR_TCIE;
                    }
                    else {
                        mcuDmaChannel->CCR &= ~DMA_CCR_TCIE;
                    }
                    enable(true);
                }
                template<bool Enable = true>
                static inline void setTCIsr() {
                    if constexpr(Enable) {
                        mcuDmaChannel->CCR |= DMA_CCR_TCIE;
                    }
                    else {
                        mcuDmaChannel->CCR &= ~DMA_CCR_TCIE;
                    }
                }
                template<bool autoEnable = true>
                static inline void reenable(const auto f) {
                    enable(false);
                    f();
                    if (autoEnable) {
                        enable(true);
                    }
                }
                template<bool autoEnable = true>
                static inline void count(const uint16_t n) {
                    enable(false);
                    mcuDmaChannel->CNDTR = n;
                    if (autoEnable) {
                        enable(true);
                    }
                }
                static inline void clearTransferCompleteIF() {
                    controller::mcuDma->IFCR = 0x1UL << (4 * (N - 1) + 1);
                }
                static inline void onTransferComplete(auto f) {
                    if (controller::mcuDma->ISR & (0x1UL << (4 * (N - 1) + 1))) {
                        clearTransferCompleteIF();
                        f();
                    }
                }
            };

        }

    }

    template<typename MCU>
    requires(G4xx<MCU> || G0xx<MCU>)
    struct Address<Dma::Controller<1, MCU>> {
        static inline constexpr uintptr_t value = DMA1_BASE;
    };
#ifdef STM32G4
    template<G4xx MCU>
    struct Address<Dma::Controller<2, MCU>> {
        static inline constexpr uintptr_t value = DMA2_BASE;
    };
#endif
#ifdef STM32G0B1xx
    template<G0xx MCU>
    struct Address<Dma::Controller<2, MCU>> {
        static inline constexpr uintptr_t value = DMA2_BASE;
    };
#endif
    template<typename MCU>
    requires(G4xx<MCU> || G0xx<MCU>)
    struct Address<Dma::Channel<Dma::Controller<1, MCU>, 1, MCU>> {
        static inline constexpr uintptr_t value = DMA1_Channel1_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel0_BASE;
    };
    template<typename MCU>
    requires(G4xx<MCU> || G0xx<MCU>)
    struct Address<Dma::Channel<Dma::Controller<1, MCU>, 2, MCU>> {
        static inline constexpr uintptr_t value = DMA1_Channel2_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel1_BASE;
    };
    template<typename MCU>
    requires(G4xx<MCU> || G0xx<MCU>)
    struct Address<Dma::Channel<Dma::Controller<1, MCU>, 3, MCU>> {
        static inline constexpr uintptr_t value = DMA1_Channel3_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel2_BASE;
    };
    template<typename MCU>
    requires(G4xx<MCU> || G0xx<MCU>)
    struct Address<Dma::Channel<Dma::Controller<1, MCU>, 4, MCU>> {
        static inline constexpr uintptr_t value = DMA1_Channel4_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel3_BASE;
    };
    template<typename MCU>
    requires(G4xx<MCU> || G0xx<MCU>)
    struct Address<Dma::Channel<Dma::Controller<1, MCU>, 5, MCU>> {
        static inline constexpr uintptr_t value = DMA1_Channel5_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel4_BASE;
    };

#ifdef DMA1_Channel6_BASE
    template<typename MCU>
    requires(G4xx<MCU> || G0xx<MCU>)
    struct Address<Dma::Channel<Dma::Controller<1, MCU>, 6, MCU>> {
        static inline constexpr uintptr_t value = DMA1_Channel6_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel5_BASE;
    };
#endif
#ifdef DMA1_Channel7_BASE
    template<typename MCU>
    requires(G4xx<MCU> || G0xx<MCU>)
    struct Address<Dma::Channel<Dma::Controller<1, MCU>, 7, MCU>> {
        static inline constexpr uintptr_t value = DMA1_Channel7_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel6_BASE;
    };
#endif
#ifdef STM32G4
    template<G4xx MCU>
    struct Address<Dma::Channel<Dma::Controller<1, MCU>, 6, MCU>> {
        static inline constexpr uintptr_t value = DMA1_Channel6_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel5_BASE;
    };
#endif

#if defined(STM32G4)
    template<G4xx MCU>
    struct Address<Dma::Channel<Dma::Controller<2, MCU>, 1, MCU>> {
        static inline constexpr uintptr_t value = DMA2_Channel1_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel6_BASE;
    };
    template<G4xx MCU>
    struct Address<Dma::Channel<Dma::Controller<2, MCU>, 2, MCU>> {
        static inline constexpr uintptr_t value = DMA2_Channel2_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel7_BASE;
    };
    template<G4xx MCU>
    struct Address<Dma::Channel<Dma::Controller<2, MCU>, 3, MCU>> {
        static inline constexpr uintptr_t value = DMA2_Channel3_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel8_BASE;
    };
    template<G4xx MCU>
    struct Address<Dma::Channel<Dma::Controller<2, MCU>, 4, MCU>> {
        static inline constexpr uintptr_t value = DMA2_Channel4_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel9_BASE;
    };
    template<G4xx MCU>
    struct Address<Dma::Channel<Dma::Controller<2, MCU>, 5, MCU>> {
        static inline constexpr uintptr_t value = DMA2_Channel5_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel10_BASE;
    };
    template<G4xx MCU>
    struct Address<Dma::Channel<Dma::Controller<2, MCU>, 6, MCU>> {
        static inline constexpr uintptr_t value = DMA2_Channel6_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel11_BASE;
    };
#endif

#if defined(STM32G0B1xx)
    template<G0xx MCU>
    struct Address<Dma::Channel<Dma::Controller<2, MCU>, 1, MCU>> {
        static inline constexpr uintptr_t value = DMA2_Channel1_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel7_BASE;
    };
    template<G0xx MCU>
    struct Address<Dma::Channel<Dma::Controller<2, MCU>, 2, MCU>> {
        static inline constexpr uintptr_t value = DMA2_Channel2_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel8_BASE;
    };
    template<G0xx MCU>
    struct Address<Dma::Channel<Dma::Controller<2, MCU>, 3, MCU>> {
        static inline constexpr uintptr_t value = DMA2_Channel3_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel9_BASE;
    };
    template<G0xx MCU>
    struct Address<Dma::Channel<Dma::Controller<2, MCU>, 4, MCU>> {
        static inline constexpr uintptr_t value = DMA2_Channel4_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel10_BASE;
    };
    template<G0xx MCU>
    struct Address<Dma::Channel<Dma::Controller<2, MCU>, 5, MCU>> {
        static inline constexpr uintptr_t value = DMA2_Channel5_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel11_BASE;
    };
#endif
}
