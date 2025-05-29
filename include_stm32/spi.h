#pragma once

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "components.h"
#include "units.h"
#include "concepts.h"
#include "dma_2.h"

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

            using value_type = Config::value_type;

            using Ch_R = Config::rxDmaComponent;
            using Ch_W = Config::txDmaComponent;

            struct dmaRConfig {
                using value_t = value_type;
                using controller = Mcu::Stm::Dma::Controller<Ch_R::controller::number_t::value>;
                static inline constexpr bool memoryIncrement = true;
            };
            using dmaChR = Mcu::Stm::Dma::V2::Channel<Ch_R::number_t::value, dmaRConfig>;
            struct dmaWConfig {
                using value_t = value_type;
                using controller = Mcu::Stm::Dma::Controller<Ch_W::controller::number_t::value>;
                static inline constexpr bool memoryIncrement = true;
            };
            using dmaChW = Mcu::Stm::Dma::V2::Channel<Ch_W::number_t::value, dmaWConfig>;

            static inline void init() {
                if constexpr(N == 1) {
#ifdef STM32G0
                    RCC->APBENR2 |= RCC_APBENR2_SPI1EN;
#else
                    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
#endif

                    dmaChR::init();
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
                            // r |= SPI_CR2_TXEIE;
                            r |= SPI_CR2_SSOE;
                            r |= SPI_CR2_RXDMAEN;
                            r |= SPI_CR2_TXDMAEN;
                            return r;
                    }();

                    mcuSpi->CR1 = [] consteval {
                            uint32_t r = 0;
                            r |= (SPI_CR1_MSTR);
                            r |= (0b101 << SPI_CR1_BR_Pos); // div 64
                            r |= SPI_CR1_SPE;
                            return r;
                    }();
                }
                else {
                    static_assert(false);
                }
            }
            static inline void transfer() {
                dmaChR::startRead(Config::size, (uint32_t)&mcuSpi->DR, mRxData, Spis::Properties<N>::dmamux_rx_src);
                dmaChR::startWrite(Config::size, (uint32_t)&mcuSpi->DR, mTxData, Spis::Properties<N>::dmamux_tx_src);
            }
            struct Isr {
                static inline void onTransferComplete(const auto f) {
                    if (mcuSpi->SR & SPI_SR_TXE) {
                        f();
                    }
                }
            };
            private:
            static inline std::array<value_type, Config::size> mTxData{};
            static inline std::array<value_type, Config::size> mRxData{};
        };
    }
    
    template<>
    struct Address<Mcu::Components::SPI<1>> {
        static inline constexpr uintptr_t value = SPI1_BASE;
    };
}
