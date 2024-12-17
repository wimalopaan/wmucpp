#pragma once

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "units.h"
#include "concepts.h"
#include "components.h"

#include <type_traits>
#include <concepts>

namespace Mcu::Stm {
    namespace Dma {
        using namespace Units::literals;

        template<uint8_t N, typename MCU = void> struct Controller;

        template<uint8_t N, typename MCU>
            requires (N >= 1) && (N <= 2)
        struct Controller<N, MCU> {
                using component_t = Mcu::Components::Dma<N>;

                // static inline /*constexpr */ DMA_TypeDef* const mcuDma = reinterpret_cast<DMA_TypeDef*>(Mcu::Stm::Address<Controller<N, MCU>>::value);
                static inline /*constexpr */ DMA_TypeDef* const mcuDma = reinterpret_cast<DMA_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Dma<N>>::value);

#ifdef STM32G4
            static inline void init() {
                if constexpr (N == 1) {
                    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
                }
                else if constexpr (N == 2) {
                    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
                }
                else {
                    static_assert(false);
                }
            }
#endif
#ifdef STM32G0
            static inline void init() {
                if constexpr (N == 1) {
                    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
                }
#ifdef STM32G0B1xx
                else if constexpr(N == 2) {
                    RCC->AHBENR |= RCC_AHBENR_DMA2EN;
                }
#endif
                else {
                    static_assert(false);
                }
            }
#endif
        };

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

    template<>
    struct Address<Mcu::Components::Dma<1>> {
        static inline constexpr uintptr_t value = DMA1_BASE;
    };
#ifdef DMA2_BASE
    template<>
    struct Address<Mcu::Components::Dma<2>> {
        static inline constexpr uintptr_t value = DMA2_BASE;
    };
#endif

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

    template<>
    struct Address<Mcu::Components::DmaChannel<Mcu::Components::Dma<2>, 4>> {
        static inline constexpr uintptr_t value = DMA2_Channel4_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel10_BASE;
    };
    template<>
    struct Address<Mcu::Components::DmaChannel<Mcu::Components::Dma<2>, 5>> {
        static inline constexpr uintptr_t value = DMA2_Channel5_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel11_BASE;
    };

#endif
}
