#pragma once

namespace Mcu::Stm {
    template<>
    struct Address<Mcu::Components::DmaChannel<Mcu::Components::Dma<1>, 1>> {
        static inline constexpr uintptr_t value = DMA1_Channel1_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel0_BASE;
    };
    template<>
    struct Address<Mcu::Components::DmaChannel<Mcu::Components::Dma<1>, 2>> {
        static inline constexpr uintptr_t value = DMA1_Channel2_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel1_BASE;
    };
    template<>
    struct Address<Mcu::Components::DmaChannel<Mcu::Components::Dma<1>, 3>> {
        static inline constexpr uintptr_t value = DMA1_Channel3_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel2_BASE;
    };
    template<>
    struct Address<Mcu::Components::DmaChannel<Mcu::Components::Dma<1>, 4>> {
        static inline constexpr uintptr_t value = DMA1_Channel4_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel3_BASE;
    };
    template<>
    struct Address<Mcu::Components::DmaChannel<Mcu::Components::Dma<1>, 5>> {
        static inline constexpr uintptr_t value = DMA1_Channel5_BASE;
        static inline constexpr uintptr_t mux = DMAMUX1_Channel4_BASE;
    };
}
