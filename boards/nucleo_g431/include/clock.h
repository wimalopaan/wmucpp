#pragma once

#include <type_traits>
#include <concepts>
#include <chrono>

#include "units.h"
#include "concepts.h"
#include "mcu_traits.h"

namespace Mcu::Stm {
    using namespace Units::literals;
    
    struct HSI;
    
    template<Units::megahertz Freq, Units::hertz SysTickFreq = 1000_Hz, typename ClockSource = HSI> struct ClockConfig; 

    template<Units::hertz SysTickFreq>
    struct ClockConfig<170_MHz, SysTickFreq, HSI> {
        static inline constexpr uint8_t pllM{4 - 1}; // 4
        static inline constexpr uint8_t pllN{85}; // 85
        static inline constexpr uint8_t pllR{0}; // 2
        static inline constexpr uint8_t pllP{2};
        static inline constexpr uint8_t pllQ{2};
        
        static inline constexpr Units::megahertz f{170_MHz};
        static inline constexpr Units::hertz frequency = f;
        static inline constexpr uint32_t systick{f / SysTickFreq}; 
//        std::integral_constant<uint32_t, systick>::_;
        
        static inline constexpr std::chrono::microseconds systickIntervall{1'000'000 / SysTickFreq.value};
    };
    
    template<typename Config, typename MCU = void> struct Clock;
    template<typename Config, Mcu::Stm::G4xx MCU>
    struct Clock<Config, MCU> {
        using config = Config;
        static inline void init() {
            RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;            
            RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
            
            RCC->CFGR |= RCC_CFGR_HPRE_DIV2;            
            
//            FLASH->ACR |= FLASH_ACR_LATENCY_5WS 
            FLASH->ACR |= FLASH_ACR_LATENCY_4WS // sollte laut DB reichen
                          | FLASH_ACR_ICEN 
                          | FLASH_ACR_DCEN 
                          | FLASH_ACR_PRFTEN;
            
            PWR->CR5 &= PWR_CR5_R1MODE;
            PWR->CR1 |= PWR_CR1_VOS_0;
            
            RCC->CR |= RCC_CR_HSION;
            while (!(RCC->CR & RCC_CR_HSIRDY));
            
            RCC->PLLCFGR |= (Config::pllM << RCC_PLLCFGR_PLLM_Pos) 
                            | (Config::pllR << RCC_PLLCFGR_PLLR_Pos)
                            | (Config::pllN << RCC_PLLCFGR_PLLN_Pos)
                            | (Config::pllP << RCC_PLLCFGR_PLLPDIV_Pos)
                            | (Config::pllQ << RCC_PLLCFGR_PLLQ_Pos)
                            | RCC_PLLCFGR_PLLSRC_HSI | RCC_PLLCFGR_PLLREN;
            
            RCC->CR |= RCC_CR_PLLON;
            while (!(RCC->CR & RCC_CR_PLLRDY));
            
            RCC->CFGR |= RCC_CFGR_SW_PLL;
            
            while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);

            MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk, RCC_CFGR_HPRE_DIV1);            
        }        
    };

    template<typename Clock,  typename UseInterrupts = std::false_type, typename MCU = void> struct SystemTimer;
    template<typename Clock, Concept::Flag UseInterrupts, Mcu::Stm::G4xx MCU>
    struct SystemTimer<Clock, UseInterrupts, MCU> {
        static inline constexpr std::chrono::microseconds intervall = Clock::config::systickIntervall;
        
        static inline constexpr Units::hertz frequency{1'000'000 / intervall.count()};
        
        inline static void init() {
            SysTick->LOAD = Clock::config::systick;
            SysTick->CTRL |= (1 << SysTick_CTRL_ENABLE_Pos) 
                             | (1 << SysTick_CTRL_CLKSOURCE_Pos);

            if constexpr(UseInterrupts::value) {
                SysTick->CTRL |= (1 << SysTick_CTRL_TICKINT_Pos);                
            }
        }
        template<typename F = UseInterrupts, typename = std::enable_if_t<!F::value>>
        inline static void periodic(const auto f) {
            if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
                ++mValue;
                f();
            }
        }
        template<typename F = UseInterrupts, typename = std::enable_if_t<F::value>>
        inline static void 
        isr() {
            ++mValue;
        }
        inline static void test() {
            periodic();
        }
    private:
//        inline static Interrupt::volatile_atomic<uint32_t> mValue{0};
        inline static uint32_t mValue{0};
    public:
        inline static volatile auto& value{mValue};
    };
}

