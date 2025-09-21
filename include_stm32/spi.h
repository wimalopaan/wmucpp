#pragma once

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "components.h"
#include "units.h"
#include "concepts.h"
#include "dma_dual_2.h"

#include <type_traits>
#include <concepts>

namespace Mcu::Stm {
    namespace Spis {
        template<uint8_t N> struct Properties;
#ifdef STM32G0
        template<> struct Properties<1> {
            static inline constexpr uint8_t dmamux_rx_src = 16;
            static inline constexpr uint8_t dmamux_tx_src = 17;
        };
#endif
    }
}
namespace Mcu::Stm {
    using namespace Units::literals;
#ifdef USE_MCU_STM_V2
    inline
#endif
    namespace V2 {
        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct Spi {
            using component_t = Mcu::Components::SPI<N>;
            static inline /*constexpr */ SPI_TypeDef* const mcuSpi= reinterpret_cast<SPI_TypeDef*>(Mcu::Stm::Address<component_t>::value);
            static inline constexpr uint8_t number = N;

            static inline constexpr std::array<std::pair<uint16_t, uint8_t>, 8> clockDividerBits = {
                std::pair{2,   0b000},
                std::pair{4,   0b001},
                std::pair{8,   0b010},
                std::pair{16,  0b011},
                std::pair{32,  0b100},
                std::pair{64,  0b101},
                std::pair{128, 0b110},
                std::pair{256, 0b111},
            };
            static inline constexpr uint16_t clockDivider = Config::clockDivider;

            using value_type = Config::value_type;
            using Ch_R = Config::rxDmaComponent;
            using Ch_W = Config::txDmaComponent;

            static inline constexpr bool txOnly = !std::is_same_v<Ch_W, void> && std::is_same_v<Ch_R, void>;
            static inline constexpr bool rxOnly = std::is_same_v<Ch_W, void> && !std::is_same_v<Ch_R, void>;

            struct dmaWConfig {
                using debug = Config::debug;
                using value_t = value_type;
                using controller = Mcu::Stm::Dma::Controller<Ch_W::controller::number_t::value>;
                static inline constexpr bool memoryIncrement = true;
                struct Isr {
                    static inline constexpr bool txComplete = Config::Isr::txComplete;
                };
            };
            using dmaChW = Mcu::Stm::Dma::V2::DualChannel<void, void, Ch_W, dmaWConfig>;

            static inline void init() {
                if constexpr(N == 1) {
#ifdef STM32G0
                    RCC->APBENR2 |= RCC_APBENR2_SPI1EN;
#else
                    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
#endif
                    // dmaChR::init();
                    dmaChW::init();
                    mcuSpi->CR2 = [] consteval {
                            uint32_t r = 0;
                            if constexpr(std::is_same_v<value_type, uint8_t>) {
                                r |= (0b0111 << SPI_CR2_DS_Pos);
                            }
                            else if constexpr(std::is_same_v<value_type, uint16_t>) {
                                r |= (0b1111 << SPI_CR2_DS_Pos);
                            }
                            else {
                                static_assert(false);
                            }
                            r |= SPI_CR2_SSOE;
                            r |= SPI_CR2_NSSP;
                            r |= SPI_CR2_RXDMAEN;
                            r |= SPI_CR2_TXDMAEN;
                            return r;
                    }();
                    mcuSpi->CR1 = [] consteval {
                            uint32_t r = 0;
                            r |= (SPI_CR1_MSTR);
                            // r |= (0b101 << SPI_CR1_BR_Pos); // div 64
                            bool found = false;
                            for(const auto& b : clockDividerBits) {
                                if (b.first == clockDivider) {
                                    r |= ((b.second & 0b111) << SPI_CR1_BR_Pos);
                                    found = true;
                                    break;
                                }
                            }
                            void f();
                            if (!found) f();
                            r |= SPI_CR1_SPE;
                            return r;
                    }();
                }
                else {
                    static_assert(false);
                }
            }
            static inline void fillSendBuffer(auto f) {
                const uint16_t n = f(mTxData);
                // dmaChR::startRead(n, (uint32_t)&mcuSpi->DR, &mRxData[0], Spis::Properties<N>::dmamux_rx_src);
                dmaChW::startWrite(n, (uint32_t)&mcuSpi->DR, &mTxData[0], Spis::Properties<N>::dmamux_tx_src);
            }
            struct Isr {
                static inline void onTransferComplete(const auto f) {
                    dmaChW::Isr::onTransferComplete(f);
                }
            };
            private:
            static inline std::array<volatile value_type, Config::size> mTxData{};
            // static inline std::array<volatile value_type, Config::size> mRxData{};
        };
    }    
    template<>
    struct Address<Mcu::Components::SPI<1>> {
        static inline constexpr uintptr_t value = SPI1_BASE;
    };
}
