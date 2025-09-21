/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "mcu/mcu.h"
#include "units.h"
#include "concepts.h"
#include "meta.h"
#include "mcu/mcu_traits.h"
#include "components.h"

#include "dma_dual_2.h"

#include <type_traits>
#include <concepts>

namespace Mcu::Stm {
    using namespace Units::literals;

    struct EndOfSequence;
    struct EndOfConversion;

    struct NoTriggerSource;
    template<auto N>
    struct ContinousSampling {
        static inline constexpr auto clockDivider = N;
    };
    
    template<bool V>
    struct UseDma : std::integral_constant<bool, V> {};

    template<typename T>
    concept isTriggerSource = requires(T t) {
    {T::trgo()};
};
    template<typename T>
    concept isClockDivider = requires(T t) {
    {T::clockDivider};
};

    template<typename C>
    struct isContinousSampling : std::false_type {};
    template<auto N>
    struct isContinousSampling<ContinousSampling<N>> : std::true_type {};

    namespace Adcs {
        template<uint8_t N> struct Properties;
        template<> struct Properties<1> {
            static inline constexpr uint8_t dmamux_src{5};
        };
#ifdef STM32G4
        template<> struct Properties<2> {
            static inline constexpr uint8_t dmamux_src{36};
        };
#endif
    }

    namespace V4 {
#define TEMPSENSOR_CAL1_ADDR               ((uint16_t*) (0x1FFF75A8UL))
#define TEMPSENSOR_CAL2_ADDR               ((uint16_t*) (0x1FFF75CAUL))
#define TEMPSENSOR_CAL1_TEMP               (30L)
#define TEMPSENSOR_CAL2_TEMP               (110L)

        static inline float adc2Temp(const uint16_t v) {
            return (float)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / (*TEMPSENSOR_CAL2_ADDR - *TEMPSENSOR_CAL1_ADDR) * (v - *TEMPSENSOR_CAL1_ADDR) + TEMPSENSOR_CAL1_TEMP;
        }

        template<uint8_t N, typename Config, typename MCU = DefaultMcu> struct Adc;
        template<uint8_t N, typename Config, typename MCU>
        requires ((N >= 1) && (N <= 2))
        struct Adc<N, Config, MCU> {
            static inline /*constexpr */ ADC_TypeDef* const mcuAdc = reinterpret_cast<ADC_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Adc<N, MCU>>::value);
            static inline /*constexpr */ ADC_Common_TypeDef* const mcuAdcCommon = reinterpret_cast<ADC_Common_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Adc<N, MCU>>::common);

            static inline constexpr uint8_t number = N;
#ifdef STM32G4
            static inline constexpr std::array<uint8_t, 4> sqr1Positions{ADC_SQR1_SQ1_Pos, ADC_SQR1_SQ2_Pos, ADC_SQR1_SQ3_Pos, ADC_SQR1_SQ4_Pos};
            static inline constexpr std::array<uint8_t, 5> sqr2Positions{ADC_SQR2_SQ5_Pos, ADC_SQR2_SQ6_Pos, ADC_SQR2_SQ7_Pos, ADC_SQR2_SQ8_Pos, ADC_SQR2_SQ9_Pos};
#endif
            using channels_list = Config::channels;
            static inline constexpr uint8_t nChannels = channels_list::size();
            static_assert(nChannels <= 9);
            static_assert(nChannels > 0);

            using dmaChComponent = Config::dmaChannel;
            struct dmaChConfig {
                using debug = Config::debug;
                using controller = Mcu::Stm::Dma::Controller<dmaChComponent::controller::number_t::value>;
                using value_t = uint16_t;
                static inline constexpr bool memoryIncrement = true;
                static inline constexpr bool circular = true;
            };
            using dmaChannel = Mcu::Stm::Dma::V2::Channel<dmaChComponent::number_t::value, dmaChConfig>;

            using dmaStorage = std::array<volatile uint16_t, nChannels>;

            using trigger = Config::trigger;

            using isrConfig = Config::isrConfig;

            static inline void wait_us(const uint32_t us) {
                volatile uint32_t w = us * 170;
                while(w != 0) {
                    w = w - 1;
                }
            }

// #ifdef STM32G4
#if 0
            static inline void init() {
                RCC->CCIPR |= 0x02 << RCC_CCIPR_ADC12SEL_Pos; // System Clock
                RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;

                MODIFY_REG(mcuAdc->CR , ADC_CR_DEEPPWD_Msk, 0x00 << ADC_CR_DEEPPWD_Pos);
                mcuAdc->CR |= ADC_CR_ADVREGEN;

                wait_us(100); // really???

                mcuAdc->ISR = ADC_ISR_ADRDY; // clear flag

                MODIFY_REG(mcuAdcCommon->CCR, ADC_CCR_PRESC_Msk, (0b0010 << ADC_CCR_PRESC_Pos)); // 170 / 4 = 42,5 MHz

                mcuAdcCommon->CCR |= ADC_CCR_VSENSESEL; // temp
                mcuAdcCommon->CCR |= ADC_CCR_VREFEN; // temp

                MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_RES_Msk, (0x00 << ADC_CFGR_RES_Pos)); // 12 bit

                if constexpr(!std::is_same_v<TriggerSource, NoTriggerSource>) {
                    // TriggerSource::_;
                    // std::integral_constant<uint16_t, TriggerSource::trgo()>::_;
                    MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_EXTSEL_Msk, (TriggerSource::trgo() << ADC_CFGR_EXTSEL_Pos));
                    MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_EXTEN_Msk, (0b10 << ADC_CFGR_EXTEN_Pos)); // falling
                    // MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_EXTEN_Msk, (0b11 << ADC_CFGR_EXTEN_Pos)); // both
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

                mcuAdc->SQR1 = []{
                    uint32_t r = (nChannels - 1) << ADC_SQR1_L_Pos;
                    for(uint8_t i{0}; i < std::min(nChannels, uint8_t{4}); ++i) {
                        r |= (channels[i] << sqr1Positions[i]);
                    }
                    return r;
                }();

                if constexpr(nChannels > 4) {
                    mcuAdc->SQR2 = []{
                        uint32_t r = 0;
                        for(uint8_t i{4}; i < nChannels; ++i) {
                            r |= (channels[i] << sqr2Positions[i - 4]);
                        }
                        return r;
                    }();
                }
                if constexpr(!std::is_same_v<ISRConfig, void>) {
                    if constexpr(Meta::contains_v<ISRConfig, EndOfSequence>) {
                        mcuAdc->IER |= ADC_IER_EOSIE;
                    }
                    if constexpr(Meta::contains_v<ISRConfig, EndOfConversion>) {
                        mcuAdc->IER |= ADC_IER_EOCIE;
                    }
                }

                mcuAdc->CR |= ADC_CR_ADCAL;
                while(mcuAdc->CR & ADC_CR_ADCAL);

                wait_us(1); // 4 clock cycles needed (see Datasheet)

                mcuAdc->CR |= ADC_CR_ADEN;
            }
#endif
#ifdef STM32G0
            static inline void reset() {
                RCC->APBRSTR2 = RCC_APBRSTR2_ADCRST;
                RCC->APBRSTR2 &= ~RCC_APBRSTR2_ADCRST;
            }
            static inline void init() {
                RCC->APBENR2 |= RCC_APBENR2_ADCEN;
                mcuAdc->CR |= ADC_CR_ADVREGEN;

                wait_us(100);

                mcuAdc->ISR = -1;

                mcuAdcCommon->CCR = [] consteval {
                        uint32_t r = 0;
                        if constexpr(isClockDivider<trigger>) {
                            static constexpr uint16_t values[16] = {1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256};
                            static constexpr uint32_t bits = [&]{
                                for(uint8_t i = 0; i < std::size(values); ++i) {
                                    if (values[i] == trigger::clockDivider) {
                                        return i;
                                    }
                                }
                                assert(false);
                            }();
                            // std::integral_constant<uint8_t, bits>::_;
                            r |= (bits << ADC_CCR_PRESC_Pos); // 64 / 4 = 42,5 MHz
                        }
                        else {
                            r |= (0b0010UL << ADC_CCR_PRESC_Pos); // 64 / 4 = 42,5 MHz
                        }
                        r |= ADC_CCR_VREFEN;
                        r |= ADC_CCR_TSEN;
                        return r;
                }();

                mcuAdc->CHSELR = []<auto... CC>(std::integer_sequence<auto, CC...>) consteval {
                        if constexpr(false) {
                        // if constexpr(sizeof...(CC) <= 8) {
                            uint32_t r = 0;
                            const std::array<uint8_t, sizeof...(CC)> channels{CC...};
                            for(uint8_t i = 0; i < 8; ++i) {
                                if (i < channels.size()) {
                                    r |= (uint32_t(channels[i] & 0x0f) << (4 * i));
                                }
                                else {
                                    r |= (uint32_t{0b1111} << (4 * i));
                                }
                            }
                            return r;
                        }
                        else {
                            return ((uint32_t{1} << CC) | ...);
                        }
                    }(channels_list{});

                mcuAdc->CFGR1 = [] consteval {
                        uint32_t r = 0;
                        r |= (0x00 << ADC_CFGR1_RES_Pos);
                        if constexpr(isTriggerSource<trigger>) {
                            r |= (trigger::trgo() << ADC_CFGR1_EXTSEL_Pos);
                            r |= (0b10UL << ADC_CFGR1_EXTEN_Pos); // falling
                        }
                        if constexpr(isContinousSampling<trigger>::value) {
                            r |= ADC_CFGR1_CONT;
                        }
                        r |= (ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN);
                        // if constexpr(nChannels <= 8) {
                        //     r |= ADC_CFGR1_CHSELRMOD;
                        // }
                        return r;
                }();

                while(!(mcuAdc->ISR & ADC_ISR_CCRDY));

                dmaChannel::init();
                dmaChannel::startRead(nChannels, (uint32_t)&mcuAdc->DR, &mData[0], Mcu::Stm::Adcs::Properties<N>::dmamux_src);

                mcuAdc->IER = [] consteval {
                        uint32_t r = 0;
                        if constexpr(Meta::contains_v<isrConfig, EndOfSequence>) {
                            r |= ADC_IER_EOSIE;
                        }
                        if constexpr(Meta::contains_v<isrConfig, EndOfConversion>) {
                            r |= ADC_IER_EOCIE;
                        }
                        return r;
                }();

                mcuAdc->CR |= ADC_CR_ADCAL;
                while(mcuAdc->CR & ADC_CR_ADCAL);

                wait_us(1); // Just to be safe

                mcuAdc->CR |= ADC_CR_ADEN;
            }
#endif
            struct Isr {
                static inline void onEnd(auto f) {
                    if constexpr(Meta::contains_v<isrConfig, EndOfSequence>) {
                        if (mcuAdc->ISR & ADC_ISR_EOS) {
                            mcuAdc->ISR = ADC_ISR_EOS;
                            f();
                        }
                    }
                    if constexpr(Meta::contains_v<isrConfig, EndOfConversion>) {
                        if (mcuAdc->ISR & ADC_ISR_EOC) {
                            mcuAdc->ISR = ADC_ISR_EOC;
                            f();
                        }
                    }
                }
            };
#ifdef STM32G0
            static inline void oversample(const uint8_t n) {
                if (n > 0) {
                    const uint8_t r = n - 1;
                    MODIFY_REG(mcuAdc->CFGR2, (ADC_CFGR2_OVSE_Msk | ADC_CFGR2_OVSS_Msk | ADC_CFGR2_OVSR_Msk),
                               (1 << ADC_CFGR2_OVSE_Pos) | (n << ADC_CFGR2_OVSS_Pos) | (r << ADC_CFGR2_OVSR_Pos));
                }
            }
#endif
            static inline void start() {
                mcuAdc->CR |= ADC_CR_ADSTART;
            }
            static inline bool ready() {
                return mcuAdc->ISR & ADC_ISR_ADRDY;
            }
            static inline bool busy() {
                return !((mcuAdc->ISR & ADC_ISR_EOC) || (mcuAdc->ISR & ADC_ISR_EOS));
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
            static inline const auto& values() {
                return mData;
            }
            private:
            static inline dmaStorage mData;
        };
    }

#ifdef USE_MCU_STM_V3
    inline
#endif
    namespace V3 {
#define TEMPSENSOR_CAL1_ADDR               ((uint16_t*) (0x1FFF75A8UL))
#define TEMPSENSOR_CAL2_ADDR               ((uint16_t*) (0x1FFF75CAUL))
#define TEMPSENSOR_CAL1_TEMP               (30L)
#define TEMPSENSOR_CAL2_TEMP               (110L)

        static inline float adc2Temp(const uint16_t v) {
            return (float)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / (*TEMPSENSOR_CAL2_ADDR - *TEMPSENSOR_CAL1_ADDR) * (v - *TEMPSENSOR_CAL1_ADDR) + TEMPSENSOR_CAL1_TEMP;
        }

        template<uint8_t N, typename ChannelList, typename TriggerSource = void, typename DmaChannel = void, typename DmaStorage = void, typename ISRConfig = void, typename MCU = DefaultMcu>
        struct Adc;

        template<uint8_t N, auto... Channels, typename TriggerSource, typename DmaChannel, typename DmaStorage, typename ISRConfig, typename MCU>
        requires ((N >= 1) && (N <= 2))
        struct Adc<N, Meta::NList<Channels...>, TriggerSource, DmaChannel, DmaStorage, ISRConfig, MCU> {
            static inline /*constexpr */ ADC_TypeDef* const mcuAdc = reinterpret_cast<ADC_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Adc<N, MCU>>::value);
            static inline /*constexpr */ ADC_Common_TypeDef* const mcuAdcCommon = reinterpret_cast<ADC_Common_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Adc<N, MCU>>::common);

#ifdef STM32G4
            static inline constexpr std::array<uint8_t, 4> sqr1Positions{ADC_SQR1_SQ1_Pos, ADC_SQR1_SQ2_Pos, ADC_SQR1_SQ3_Pos, ADC_SQR1_SQ4_Pos};
            static inline constexpr std::array<uint8_t, 5> sqr2Positions{ADC_SQR2_SQ5_Pos, ADC_SQR2_SQ6_Pos, ADC_SQR2_SQ7_Pos, ADC_SQR2_SQ8_Pos, ADC_SQR2_SQ9_Pos};
#endif
            static inline constexpr uint8_t nChannels = sizeof...(Channels);
            static_assert(nChannels <= 9);
            static inline constexpr std::array<uint8_t, nChannels> channels{Channels...};

            using dmaChannel = DmaChannel;

            static inline void wait_us(const uint32_t us) {
                volatile uint32_t w = us * 170;
                while(w != 0) {
                    w = w - 1;
                }
            }

#ifdef STM32G4
            static inline void init() {
                RCC->CCIPR |= 0x02 << RCC_CCIPR_ADC12SEL_Pos; // System Clock
                RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;

                MODIFY_REG(mcuAdc->CR , ADC_CR_DEEPPWD_Msk, 0x00 << ADC_CR_DEEPPWD_Pos);
                mcuAdc->CR |= ADC_CR_ADVREGEN;

                wait_us(100); // really???

                mcuAdc->ISR = ADC_ISR_ADRDY; // clear flag

                MODIFY_REG(mcuAdcCommon->CCR, ADC_CCR_PRESC_Msk, (0b0010 << ADC_CCR_PRESC_Pos)); // 170 / 4 = 42,5 MHz

                mcuAdcCommon->CCR |= ADC_CCR_VSENSESEL; // temp
                mcuAdcCommon->CCR |= ADC_CCR_VREFEN; // temp

                MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_RES_Msk, (0x00 << ADC_CFGR_RES_Pos)); // 12 bit

                if constexpr(!std::is_same_v<TriggerSource, NoTriggerSource>) {
                    // TriggerSource::_;
                    // std::integral_constant<uint16_t, TriggerSource::trgo()>::_;
                    MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_EXTSEL_Msk, (TriggerSource::trgo() << ADC_CFGR_EXTSEL_Pos));
                    MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_EXTEN_Msk, (0b10 << ADC_CFGR_EXTEN_Pos)); // falling
                    // MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_EXTEN_Msk, (0b11 << ADC_CFGR_EXTEN_Pos)); // both
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

                mcuAdc->SQR1 = []{
                    uint32_t r = (nChannels - 1) << ADC_SQR1_L_Pos;
                    for(uint8_t i{0}; i < std::min(nChannels, uint8_t{4}); ++i) {
                        r |= (channels[i] << sqr1Positions[i]);
                    }
                    return r;
                }();

                if constexpr(nChannels > 4) {
                    mcuAdc->SQR2 = []{
                        uint32_t r = 0;
                        for(uint8_t i{4}; i < nChannels; ++i) {
                            r |= (channels[i] << sqr2Positions[i - 4]);
                        }
                        return r;
                    }();
                }
                if constexpr(!std::is_same_v<ISRConfig, void>) {
                    if constexpr(Meta::contains_v<ISRConfig, EndOfSequence>) {
                        mcuAdc->IER |= ADC_IER_EOSIE;
                    }
                    if constexpr(Meta::contains_v<ISRConfig, EndOfConversion>) {
                        mcuAdc->IER |= ADC_IER_EOCIE;
                    }
                }

                mcuAdc->CR |= ADC_CR_ADCAL;
                while(mcuAdc->CR & ADC_CR_ADCAL);

                wait_us(1); // 4 clock cycles needed (see Datasheet)

                mcuAdc->CR |= ADC_CR_ADEN;
            }
#endif
#ifdef STM32G0
            static inline void reset() {
                RCC->APBRSTR2 = RCC_APBRSTR2_ADCRST;
                RCC->APBRSTR2 &= ~RCC_APBRSTR2_ADCRST;
            }
            static inline void init() {
                //                RCC->CCIPR |= 0x02 << RCC_CCIPR_ADC12SEL_Pos; // System Clock
                RCC->APBENR2 |= RCC_APBENR2_ADCEN;

                //                MODIFY_REG(mcuAdc->CR , ADC_CR_DEEPPWD_Msk, 0x00 << ADC_CR_DEEPPWD_Pos);
                mcuAdc->CR |= ADC_CR_ADVREGEN;

                wait_us(100);

                mcuAdc->ISR = ADC_ISR_ADRDY; // clear flag

                if constexpr(isClockDivider<TriggerSource>) {
                    constexpr uint16_t values[16] = {1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256};
                    constexpr uint8_t bits = [&]{
                        for(uint8_t i = 0; i < std::size(values); ++i) {
                            if (values[i] == TriggerSource::clockDivider) {
                                return i;
                            }
                        }
                        assert(false);
                    }();
                    // std::integral_constant<uint8_t, bits>::_;
                    MODIFY_REG(mcuAdcCommon->CCR, ADC_CCR_PRESC_Msk, (bits << ADC_CCR_PRESC_Pos)); // 64 / 4 = 42,5 MHz
                }
                else {
                    MODIFY_REG(mcuAdcCommon->CCR, ADC_CCR_PRESC_Msk, (0b0010 << ADC_CCR_PRESC_Pos)); // 64 / 4 = 42,5 MHz
                }

                // mcuAdcCommon->CCR |= ADC_CCR_VSENSESEL; // temp
                mcuAdcCommon->CCR |= ADC_CCR_VREFEN; // temp

                MODIFY_REG(mcuAdc->CFGR1, ADC_CFGR1_RES_Msk, (0x00 << ADC_CFGR1_RES_Pos)); // 12 bit

                if constexpr(isTriggerSource<TriggerSource>) {
                    // TriggerSource::_;
                    // std::integral_constant<uint16_t, TriggerSource::trgo()>::_;
                    MODIFY_REG(mcuAdc->CFGR1, ADC_CFGR1_EXTSEL_Msk, (TriggerSource::trgo() << ADC_CFGR1_EXTSEL_Pos));
                    MODIFY_REG(mcuAdc->CFGR1, ADC_CFGR1_EXTEN_Msk, (0b10 << ADC_CFGR1_EXTEN_Pos)); // falling
                    // MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_EXTEN_Msk, (0b11 << ADC_CFGR_EXTEN_Pos)); // both
                }
                if constexpr(isContinousSampling<TriggerSource>::value) {
                    mcuAdc->CFGR1 |= ADC_CFGR1_CONT;
                }

                mcuAdc->CHSELR = ((1 << Channels) | ...);

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

                    mcuAdc->CFGR1 |= ADC_CFGR1_DMACFG;
                    mcuAdc->CFGR1 |= ADC_CFGR1_DMAEN;
                }

                if constexpr(!std::is_same_v<ISRConfig, void>) {
                    if constexpr(Meta::contains_v<ISRConfig, EndOfSequence>) {
                        mcuAdc->IER |= ADC_IER_EOSIE;
                    }
                    if constexpr(Meta::contains_v<ISRConfig, EndOfConversion>) {
                        mcuAdc->IER |= ADC_IER_EOCIE;
                    }
                }

                mcuAdc->CR |= ADC_CR_ADCAL;
                while(mcuAdc->CR & ADC_CR_ADCAL);

                wait_us(1); // Just to be safe

                mcuAdc->CR |= ADC_CR_ADEN;
            }
#endif
#ifdef STM32G0
            static inline void oversample(const uint8_t n) {
                if (n > 0) {
                    const uint8_t r = n - 1;
                    MODIFY_REG(mcuAdc->CFGR2, (ADC_CFGR2_OVSE_Msk | ADC_CFGR2_OVSS_Msk | ADC_CFGR2_OVSR_Msk),
                               (1 << ADC_CFGR2_OVSE_Pos) | (n << ADC_CFGR2_OVSS_Pos) | (r << ADC_CFGR2_OVSR_Pos));
                }
            }
#endif
            static inline void start() {
                mcuAdc->CR |= ADC_CR_ADSTART;
            }
            static inline bool ready() {
                return mcuAdc->ISR & ADC_ISR_ADRDY;
            }
            static inline bool busy() {
                return !((mcuAdc->ISR & ADC_ISR_EOC) || (mcuAdc->ISR & ADC_ISR_EOS));
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
            static inline const auto& values() {
                return mData;
            }
            // private:
            static inline DmaStorage mData;
            static_assert(nChannels <= mData.size());
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

#ifdef STM32G4
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
#endif
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

#ifdef STM32G4
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
#endif
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

    template<G0xx MCU>
    struct Address<Mcu::Components::Adc<1, MCU>> {
        static inline constexpr uintptr_t value = ADC1_BASE;
#ifdef STM32G0
        static inline constexpr uintptr_t common = ADC1_COMMON_BASE;
#endif
    };

    template<G4xx MCU>
    struct Address<Mcu::Components::Adc<1, MCU>> {
        static inline constexpr uintptr_t value = ADC1_BASE;
#ifdef STM32G4
        static inline constexpr uintptr_t common = ADC12_COMMON_BASE;
#endif
#ifdef STM32G0
        static inline constexpr uintptr_t common = ADC1_COMMON_BASE;
#endif
    };

#ifdef STM32G4
    template<G4xx MCU>
    struct Address<Mcu::Components::Adc<2, MCU>> {
        static inline constexpr uintptr_t value = ADC2_BASE;
        static inline constexpr uintptr_t common = ADC12_COMMON_BASE;
    };
#endif
}
