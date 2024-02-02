#pragma once

#include "mcu/mcu.h"
#include "units.h"
#include "concepts.h"
#include "meta.h"
#include "mcu/mcu_traits.h"
#include "components.h"

#include <type_traits>
#include <concepts>

namespace Mcu::Stm {
    using namespace Units::literals;

    struct NoTriggerSource;
    
    template<bool V>
    struct UseDma : std::integral_constant<bool, V> {};

    namespace Adcs {
    template<uint8_t N> struct Properties;
    template<> struct Properties<1> {
        static inline constexpr uint8_t dmamux_src{5};
    };
    template<> struct Properties<2> {
        static inline constexpr uint8_t dmamux_src{36};
    };
    }



#ifdef USE_MCU_STM_V3
    inline
#endif
    namespace V3 {

    template<uint8_t N, typename ChannelList, typename TriggerSource = void, typename DmaChannel = void, typename DmaStorage = void, typename MCU = DefaultMcu>
    struct Adc;

    template<uint8_t N, auto... Channels, typename TriggerSource, typename DmaChannel, typename DmaStorage, typename MCU>
    requires ((N >= 1) && (N <= 2))
    struct Adc<N, Meta::NList<Channels...>, TriggerSource, DmaChannel, DmaStorage, MCU> {
        static inline /*constexpr */ ADC_TypeDef* const mcuAdc = reinterpret_cast<ADC_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Adc<N, MCU>>::value);
        static inline /*constexpr */ ADC_Common_TypeDef* const mcuAdcCommon = reinterpret_cast<ADC_Common_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Adc<N, MCU>>::common);

        static inline constexpr std::array<uint8_t, 4> sqr1Positions{ADC_SQR1_SQ1_Pos, ADC_SQR1_SQ2_Pos, ADC_SQR1_SQ3_Pos, ADC_SQR1_SQ4_Pos};

        static inline constexpr uint8_t nChannels = sizeof...(Channels);
        static_assert(nChannels <= 4);
        static inline constexpr std::array<uint8_t, nChannels> channels{Channels...};

        static inline void wait_us(const uint32_t us) {
            volatile uint32_t w = us * 170;
            while(w != 0) {
                w = w - 1;
            }
        }

        static inline void init() {
            RCC->CCIPR |= 0x02 << RCC_CCIPR_ADC12SEL_Pos; // System Clock
            RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;

            MODIFY_REG(mcuAdc->CR , ADC_CR_DEEPPWD_Msk, 0x00 << ADC_CR_DEEPPWD_Pos);
            mcuAdc->CR |= ADC_CR_ADVREGEN;

            wait_us(100); // really???

            mcuAdc->ISR = ADC_ISR_ADRDY; // clear flag

            MODIFY_REG(mcuAdcCommon->CCR, ADC_CCR_PRESC_Msk, (0b0010 << ADC_CCR_PRESC_Pos)); // 170 / 4 = 42,5 MHz

            MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_RES_Msk, (0x00 << ADC_CFGR_RES_Pos)); // 12 bit

            if constexpr(!std::is_same_v<TriggerSource, NoTriggerSource>) {
                // TriggerSource::_;
                // std::integral_constant<uint16_t, TriggerSource::trgo()>::_;
                MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_EXTSEL_Msk, (TriggerSource::trgo() << ADC_CFGR_EXTSEL_Pos));
                MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_EXTEN_Msk, (0b10 << ADC_CFGR_EXTEN_Pos));
            }

            if constexpr(!std::is_same_v<DmaChannel, void>) {
                DmaChannel::init();
                DmaChannel::template msize<uint16_t>();
                DmaChannel::template psize<uint16_t>();

                DmaChannel::mcuDmaChannel->CCR |= DMA_CCR_MINC;
                DmaChannel::mcuDmaChannel->CCR |= DMA_CCR_CIRC;
                DmaChannel::mcuDmaChannel->CCR &= ~DMA_CCR_DIR;
                DmaChannel::mcuDmaChannel->CNDTR = nChannels;
                DmaChannel::mcuDmaChannel->CPAR = (uint32_t)(&mcuAdc->DR);
                DmaChannel::mcuDmaChannel->CMAR = (uint32_t)&mData[0];
                DmaChannel::enable();

                DmaChannel::mcuDmaMux->CCR = Mcu::Stm::Adcs::Properties<N>::dmamux_src & DMAMUX_CxCR_DMAREQ_ID_Msk;
                DmaChannel::mcuDmaMux->CCR |= DMAMUX_CxCR_EGE;

                DmaChannel::mcuDmaChannel->CCR |= DMA_CCR_EN;

                mcuAdc->CFGR |= ADC_CFGR_DMACFG;
                mcuAdc->CFGR |= ADC_CFGR_DMAEN;
            }

            mcuAdc->SQR1 = []<auto... II>(std::index_sequence<II...>){
                uint32_t r = (nChannels - 1) << ADC_SQR1_L_Pos;
                for(uint8_t i{0}; i < nChannels; ++i) {
                    r |= (channels[i] << sqr1Positions[i]);
                }
                return r;
            }(std::make_index_sequence<nChannels>{});

            // if constexpr(Channel == 16) { // temp
            //     mcuAdcCommon->CCR |= ADC_CCR_VSENSESEL;
            // }

            mcuAdc->CR |= ADC_CR_ADEN;
        }
        static inline void start() {
            mcuAdc->CR |= ADC_CR_ADSTART;
        }
        static inline bool ready() {
            return mcuAdc->ISR & ADC_ISR_ADRDY;
        }
        static inline bool busy() {
            return !(mcuAdc->ISR & ADC_ISR_EOC);
        }
        static inline bool gotSequence() {
            return (mcuAdc->ISR & ADC_ISR_EOS);
        }
        static inline void whenSequenceComplete(auto f) {
            if (mcuAdc->ISR & ADC_ISR_EOS) {
                mcuAdc->ISR = ADC_ISR_EOS;
                f();
            }
        }

        static inline uint16_t value() {
            return mcuAdc->DR;
        }
    // private:
        static inline DmaStorage mData;
    };

    }

#ifdef USE_MCU_STM_V2
    inline
#endif
    namespace V2 {

    template<uint8_t N, uint8_t Channel = 0, typename TriggerSource = void, typename UseDma = std::integral_constant<bool, false>, typename MCU = DefaultMcu> struct Adc;

    template<uint8_t N, uint8_t Channel, typename TriggerSource, typename DmaChannel, typename MCU>
        requires (N >= 1) && (N <=2)
        struct Adc<N, Channel, TriggerSource, DmaChannel, MCU> {
        static inline /*constexpr */ ADC_TypeDef* const mcuAdc = reinterpret_cast<ADC_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Adc<N, MCU>>::value);
        static inline /*constexpr */ ADC_Common_TypeDef* const mcuAdcCommon = reinterpret_cast<ADC_Common_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Adc<N, MCU>>::common);

        static inline void wait_us(const uint32_t us) {
            volatile uint32_t w = us * 170;
            while(w != 0) {
                w = w - 1;
            }
        }

        static inline void init() {
            RCC->CCIPR |= 0x02 << RCC_CCIPR_ADC12SEL_Pos; // System Clock
            RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;

            MODIFY_REG(mcuAdc->CR , ADC_CR_DEEPPWD_Msk, 0x00 << ADC_CR_DEEPPWD_Pos);
            mcuAdc->CR |= ADC_CR_ADVREGEN;

            wait_us(100); // really???

            mcuAdc->ISR = ADC_ISR_ADRDY; // clear flag

                   // VREF ?

                   // MODIFY_REG(mcuAdcCommon->CCR, ADC_CCR_PRESC_Msk, (0 << ADC_CCR_PRESC_Pos)); // kein
            MODIFY_REG(mcuAdcCommon->CCR, ADC_CCR_PRESC_Msk, (0b0010 << ADC_CCR_PRESC_Pos)); // 170 / 4 = 42,5 MHz
            // MODIFY_REG(mcuAdcCommon->CCR, ADC_CCR_CKMODE_Msk, (0b01 << ADC_CCR_CKMODE_Pos)); // test

            MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_RES_Msk, (0x00 << ADC_CFGR_RES_Pos)); // 12 bit

            if constexpr(!std::is_same_v<TriggerSource, NoTriggerSource>) {
                // TriggerSource::_;
                // std::integral_constant<uint16_t, TriggerSource::trgo()>::_;
                MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_EXTSEL_Msk, (TriggerSource::trgo() << ADC_CFGR_EXTSEL_Pos));
                MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_EXTEN_Msk, (0b01 << ADC_CFGR_EXTEN_Pos));
            }

            if constexpr(!std::is_same_v<DmaChannel, void>) {
                DmaChannel::init();
                DmaChannel::template msize<uint16_t>();
                DmaChannel::template psize<uint16_t>();

                DmaChannel::mcuDmaChannel->CCR |= DMA_CCR_MINC;
                DmaChannel::mcuDmaChannel->CCR |= DMA_CCR_CIRC;
                DmaChannel::mcuDmaChannel->CCR &= ~DMA_CCR_DIR;
                // dmaChannel::mcuDmaChannel->CNDTR = ;
                // dmaChannel::mcuDmaChannel->CPAR = (uint32_t)(&mcuTimer->CCR1 + N);
                // dmaChannel::mcuDmaChannel->CMAR = (uint32_t)&mValues[0];


                mcuAdc->CFGR |= ADC_CFGR_DMACFG;
                mcuAdc->CFGR |= ADC_CFGR_DMAEN;
            }

            mcuAdc->SQR1 = (0x00 << ADC_SQR1_L_Pos) | (Channel << ADC_SQR1_SQ1_Pos);
            // mcuAdc->SQR1 = (0x01 << ADC_SQR1_L_Pos) | (Channel << ADC_SQR1_SQ1_Pos) | (3 << ADC_SQR1_SQ2_Pos); // opamp1

            if constexpr(Channel == 16) { // temp
                mcuAdcCommon->CCR |= ADC_CCR_VSENSESEL;
            }

                   // mcuAdc->IER |= ADC_IER_EOCIE; // test
                   // mcuAdc->IER |= ADC_IER_EOSIE; // test

            mcuAdc->CR |= ADC_CR_ADEN;
        }
        static inline void start() {
            mcuAdc->CR |= ADC_CR_ADSTART;
        }
        static inline bool ready() {
            return mcuAdc->ISR & ADC_ISR_ADRDY;
        }
        static inline bool busy() {
            //            return mcuAdc->CR & ADC_CR_ADSTART;
            return !(mcuAdc->ISR & ADC_ISR_EOC);
        }
        static inline uint16_t value() {
            return mcuAdc->DR;
        }
    };



    template<uint8_t N, uint8_t Channel, typename TriggerSource, bool useDma, typename MCU>
        requires (N >= 1) && (N <=2)
        struct Adc<N, Channel, TriggerSource, UseDma<useDma>, MCU> {
        static inline /*constexpr */ ADC_TypeDef* const mcuAdc = reinterpret_cast<ADC_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Adc<N, MCU>>::value);
        static inline /*constexpr */ ADC_Common_TypeDef* const mcuAdcCommon = reinterpret_cast<ADC_Common_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Adc<N, MCU>>::common);

        static inline void wait_us(const uint32_t us) {
            volatile uint32_t w = us * 170;
            while(w != 0) {
                w = w - 1;
            }
        }

        static inline void init() {
            RCC->CCIPR |= 0x02 << RCC_CCIPR_ADC12SEL_Pos; // System Clock
            RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;

            MODIFY_REG(mcuAdc->CR , ADC_CR_DEEPPWD_Msk, 0x00 << ADC_CR_DEEPPWD_Pos);
            mcuAdc->CR |= ADC_CR_ADVREGEN;

            wait_us(100); // really???

            mcuAdc->ISR = ADC_ISR_ADRDY; // clear flag

            // VREF ?

            // MODIFY_REG(mcuAdcCommon->CCR, ADC_CCR_PRESC_Msk, (0 << ADC_CCR_PRESC_Pos)); // kein
            MODIFY_REG(mcuAdcCommon->CCR, ADC_CCR_PRESC_Msk, (0b0010 << ADC_CCR_PRESC_Pos)); // 170 / 4 = 42,5 MHz
            // MODIFY_REG(mcuAdcCommon->CCR, ADC_CCR_CKMODE_Msk, (0b01 << ADC_CCR_CKMODE_Pos)); // test

            MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_RES_Msk, (0x00 << ADC_CFGR_RES_Pos)); // 12 bit

            if constexpr(!std::is_same_v<TriggerSource, NoTriggerSource>) {
                // TriggerSource::_;
                // std::integral_constant<uint16_t, TriggerSource::trgo()>::_;
                // MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_EXTSEL_Msk, (TriggerSource::trgo() << ADC_CFGR_EXTSEL_Pos));
                // MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_EXTSEL_Msk, (0b00100 << ADC_CFGR_EXTSEL_Pos));
                mcuAdc->CFGR |= ADC_CFGR_EXTSEL_2;
                MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_EXTEN_Msk, (0b01 << ADC_CFGR_EXTEN_Pos));
            }

            if constexpr(useDma) {
                mcuAdc->CFGR |= ADC_CFGR_DMACFG;
                mcuAdc->CFGR |= ADC_CFGR_DMAEN;
            }

            mcuAdc->SQR1 = (0x00 << ADC_SQR1_L_Pos) | (Channel << ADC_SQR1_SQ1_Pos);
            // mcuAdc->SQR1 = (0x01 << ADC_SQR1_L_Pos) | (Channel << ADC_SQR1_SQ1_Pos) | (3 << ADC_SQR1_SQ2_Pos); // opamp1

            if constexpr(Channel == 16) { // temp
                mcuAdcCommon->CCR |= ADC_CCR_VSENSESEL;
            }

            // mcuAdc->IER |= ADC_IER_EOCIE; // test
            // mcuAdc->IER |= ADC_IER_EOSIE; // test

            mcuAdc->CR |= ADC_CR_ADEN;
        }
        static inline void start() {
            mcuAdc->CR |= ADC_CR_ADSTART;
        }
        static inline bool ready() {
            return mcuAdc->ISR & ADC_ISR_ADRDY;
        }
        static inline bool busy() {
            //            return mcuAdc->CR & ADC_CR_ADSTART;
            return !(mcuAdc->ISR & ADC_ISR_EOC);
        }
        static inline uint16_t value() {
            return mcuAdc->DR;
        }
    };

    }
    
    
    template<G4xx MCU>
    struct Address<Mcu::Components::Adc<1, MCU>> {
        static inline constexpr uintptr_t value = ADC1_BASE;
        static inline constexpr uintptr_t common = ADC12_COMMON_BASE;
    };

    template<G4xx MCU>
    struct Address<Mcu::Components::Adc<2, MCU>> {
        static inline constexpr uintptr_t value = ADC2_BASE;
        static inline constexpr uintptr_t common = ADC12_COMMON_BASE;
    };
}
