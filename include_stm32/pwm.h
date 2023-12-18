#include <type_traits>
#include <concepts>
#include <array>

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/alternate.h"
#include "units.h"
#include "concepts.h"
#include "meta.h"

#include "timer.h"

namespace Mcu::Stm {
    using namespace Units::literals;

#ifdef USE_MCU_STM_V2
    inline 
#endif
    namespace V2 {
        namespace Pwm {

            // Cyclic DMA        
            template<uint8_t TimerNumber, uint8_t MaxLength, typename Clock, typename MCU = DefaultMcu> 
            struct Sequence {
                static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Timer<TimerNumber, void, void, MCU>>::value);
            
                static inline constexpr uint16_t onems = 1640; 
                static inline constexpr uint16_t period = onems * 20; 
                static inline constexpr uint16_t prescaler = Clock::config::frequency.value / period;
                
                static inline void init() {
                    if constexpr(TimerNumber == 1) {
                        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
                    }
                    else if constexpr(TimerNumber == 2) {
                        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
                    }
                    else if constexpr(TimerNumber == 3) {
                        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
                    }
                    else if constexpr(TimerNumber == 4) {
                        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
                    }
                    else {
                        static_assert(false);
                    }
                    mcuTimer->PSC = prescaler;
                    mcuTimer->ARR = period;
                    
                    // DMA:
                    // Channel peripheral address : TIM_DMAR
                    // Memory
                    // 1 Transfer
                    // circular

                    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
                    RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
                    MODIFY_REG(DMA1_Channel1->CCR, DMA_CCR_MSIZE_Msk, 0x01 << DMA_CCR_MSIZE_Pos);
                    MODIFY_REG(DMA1_Channel1->CCR, DMA_CCR_PSIZE_Msk, 0x01 << DMA_CCR_PSIZE_Pos);
                    DMA1_Channel1->CCR |= DMA_CCR_MINC;
                    DMA1_Channel1->CCR |= DMA_CCR_CIRC;
                    DMA1_Channel1->CCR |= DMA_CCR_DIR;
                    DMA1_Channel1->CNDTR = 1;
                    DMA1_Channel1->CPAR = (uint32_t)&mcuTimer->DMAR;
                    DMA1_Channel1->CMAR = (uint32_t)&mValues[0];
                    DMA1_Channel1->CCR |= DMA_CCR_TCIE; // for test
                    DMA1_Channel1->CCR |= DMA_CCR_EN;
                    
                    // DMAMUX:
                    // Source
                    
                    DMAMUX1_Channel0->CCR = 56 & DMAMUX_CxCR_DMAREQ_ID_Msk; // timer2 ch1
                    DMAMUX1_Channel0->CCR |= DMAMUX_CxCR_EGE;
                    
                    // MODIFY_REG(DMAMUX1_RequestGenerator0->RGCR, DMAMUX_RGxCR_GPOL_Msk, 0b01 << DMAMUX_RGxCR_GPOL_Pos);
                    // DMAMUX1_RequestGenerator0->RGCR |= DMAMUX_RGxCR_GE;
                    
                    // Timer:
                    // TIM_DIER_CC1DE, ...
                    // TIM_DCR: DBA DBL
                    
                    // Enable Timer, DMA
                    
                    mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC1M_Pos); // pwm1
                    mcuTimer->CCMR1 |= TIM_CCMR1_OC1PE;
                    mcuTimer->CCER |= TIM_CCER_CC1E;
                    mcuTimer->DIER |= TIM_DIER_CC1DE;
                    MODIFY_REG(mcuTimer->DCR, TIM_DCR_DBL_Msk, (1 - 1) << TIM_DCR_DBL_Pos); // 1 transfer
                    MODIFY_REG(mcuTimer->DCR, TIM_DCR_DBA_Msk, 14 << TIM_DCR_DBA_Pos); // ccr1
                    mcuTimer->EGR |= TIM_EGR_UG;
                    mcuTimer->CR1 |= TIM_CR1_ARPE;
                    mcuTimer->CR1 |= TIM_CR1_CEN;
                }        
            private:
                static inline std::array<uint16_t, MaxLength> mValues;
            };
                
            template<typename Timer, typename OutList> struct Servo;
            
            template<typename Timer, typename... Outs>
            struct Servo<Timer, Meta::List<Outs...>> {
//                static inline constexpr uint8_t size = sizeof...(Outs);
                static inline constexpr std::integral_constant<std::size_t, sizeof...(Outs)> size{};
                
                static inline void init() {
                    Timer::init();
                }  
                static inline void duty(const std::array<uint16_t, size> d) {
                }
            };

            

            
        }
        
    }
}
