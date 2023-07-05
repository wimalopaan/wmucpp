#pragma once

#include "mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu_traits.h"

#include <type_traits>
#include <concepts>

namespace Mcu::Stm {
    using namespace Units::literals;
    
    template<uint8_t N, uint8_t Channel = 0, typename MCU = void>
    struct Adc {
        static inline /*constexpr */ ADC_TypeDef* const mcuAdc = reinterpret_cast<ADC_TypeDef*>(Mcu::Stm::Address<Adc<N, Channel, MCU>>::value);
        static inline /*constexpr */ ADC_Common_TypeDef* const mcuAdcCommon = reinterpret_cast<ADC_Common_TypeDef*>(Mcu::Stm::Address<Adc<N, Channel, MCU>>::common);
        
        static inline void wait_us(const uint32_t us) {
            volatile uint32_t w = us * 170;
            while(w != 0) {
                w = w - 1;
            }
        }
        
        static inline void init() {
            RCC->CCIPR |= 0x02 << RCC_CCIPR_ADC12SEL_Pos;
            RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
            
            MODIFY_REG(mcuAdc->CR , ADC_CR_DEEPPWD_Msk, 0x00 << ADC_CR_DEEPPWD_Pos); 
            mcuAdc->CR |= ADC_CR_ADVREGEN;
            
            wait_us(100); // really???

            mcuAdc->ISR = ADC_ISR_ADRDY; // clear flag
            
            // VREF ?
            
            mcuAdcCommon->CCR |= (0x05 << ADC_CCR_PRESC_Pos); // prescaler 10
//            mcuAdcCommon->CCR |= (0x0a << ADC_CCR_PRESC_Pos); // prescaler 10
            MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_RES_Msk, (0x00 << ADC_CFGR_RES_Pos)); // 12 bit
            
            mcuAdc->SQR1 = (0x00 << ADC_SQR1_L_Pos) | (Channel << ADC_SQR1_SQ1_Pos); // AIN10P
            
            if constexpr(Channel == 16) { // temp
                mcuAdcCommon->CCR |= ADC_CCR_VSENSESEL;
            }
            
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
    
    template<uint8_t C, G4xx MCU> 
    struct Address<Adc<1, C, MCU>> {
        static inline constexpr uintptr_t value = ADC1_BASE;
        static inline constexpr uintptr_t common = ADC12_COMMON_BASE;
    };
    template<uint8_t C, G4xx MCU> 
    struct Address<Adc<2, C, MCU>> {
        static inline constexpr uintptr_t value = ADC2_BASE;
        static inline constexpr uintptr_t common = ADC12_COMMON_BASE;
    };
    
}
