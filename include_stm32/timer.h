#pragma once

#pragma once

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "units.h"
#include "concepts.h"
#include "components.h"
#include "rcc.h"

#include <type_traits>
#include <concepts>
#include <algorithm>

namespace Mcu::Stm {
    using namespace Units::literals;

    namespace Timers {
        enum class SyncMode : uint8_t {None, Master, Slave};

        template<uint8_t N> struct Properties;
#ifdef STM32G4
        template<> struct Properties<2> {
            using value_type = uint32_t;
            static inline constexpr std::array<uint8_t, 4> dmamux_src{56, 57, 58, 59};
            static inline constexpr uint8_t dmaUpdate_src{60};
        };
        template<> struct Properties<3> {
            using value_type = uint16_t;
            static inline constexpr std::array<uint8_t, 4> dmamux_src{61, 62, 63, 64};
            static inline constexpr uint8_t dmaUpdate_src{65};
        };
        template<> struct Properties<4> {
            using value_type = uint16_t;
            static inline constexpr std::array<uint8_t, 4> dmamux_src{67, 68, 69, 70};
            static inline constexpr uint8_t dmaUpdate_src{71};
        };
        template<> struct Properties<5> {
            using value_type = uint32_t;
            static inline constexpr std::array<uint8_t, 4> dmamux_src{72, 73, 74, 75};
            static inline constexpr uint8_t dmaUpdate_src{76};
        };
        template<> struct Properties<6> {
            using value_type = uint16_t;
//            static inline constexpr std::array<uint8_t, 4> dmamux_src{72, 73, 74, 75};
           static inline constexpr uint8_t dmaUpdate_src{8};
        };
        template<> struct Properties<7> {
            using value_type = uint16_t;
//            static inline constexpr std::array<uint8_t, 4> dmamux_src{72, 73, 74, 75};
           static inline constexpr uint8_t dmaUpdate_src{9};
        };
        template<> struct Properties<16> {
            using value_type = uint16_t;
            static inline constexpr std::array<uint8_t, 4> dmamux_src{82};
            static inline constexpr uint8_t dmaUpdate_src{83};
        };
#endif
#ifdef STM32G0B1xx
        template<> struct Properties<2> {
            using value_type = uint16_t;
            static inline constexpr std::array<uint8_t, 4> dmamux_src{26, 27, 28, 29};
            static inline constexpr uint8_t dmaUpdate_src{31};
        };
        template<> struct Properties<3> {
            using value_type = uint16_t;
            static inline constexpr std::array<uint8_t, 4> dmamux_src{32, 33, 34, 35};
            static inline constexpr uint8_t dmaUpdate_src{37};
        };
        template<> struct Properties<4> {
            using value_type = uint16_t;
            static inline constexpr std::array<uint8_t, 4> dmamux_src{68, 69, 70, 71};
            static inline constexpr uint8_t dmaUpdate_src{73};
        };
        template<> struct Properties<14> {
            using value_type = uint16_t;
            enum class Trigger : uint8_t {OC = 22};
        };
#endif

        template<uint8_t Src, uint8_t Dst>
        requires ((Src >= 2) && (Src <= 5))
        uint8_t trgoToTrigger() {
            return (Src - 1);
        }

#ifdef STM32G4
        template<uint8_t TimerNumber, typename MCU = DefaultMcu>
        requires(G4xx<MCU>)
        void reset() {
            const auto rcc = Mcu::Stm::Address<Mcu::Components::Rcc>::value;
            if constexpr (TimerNumber == 1) {
                rcc->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
                rcc->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;
            }
            else if constexpr (TimerNumber == 2) {
                RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM2RST;
                RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM2RST;
            }
            else if constexpr (TimerNumber == 3) {
                RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM3RST;
                RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM3RST;
            }
            else if constexpr (TimerNumber == 4) {
                RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM4RST;
                RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM4RST;
            }
#ifdef RCC_APB1RSTR1_TIM5RST
            else if constexpr (TimerNumber == 5) {
                RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM5RST;
                RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM5RST;
            }
#endif
            else if constexpr (TimerNumber == 6) {
                RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM6RST;
                RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM6RST;
            }
            else if constexpr (TimerNumber == 7) {
                RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM7RST;
                RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM7RST;
            }
            else {
                static_assert(false);
            }
        }
#endif

#ifdef STM32G0
        template<uint8_t TimerNumber, typename MCU = DefaultMcu>
        requires(G0xx<MCU>)
        void reset() {
            const auto rcc = Mcu::Stm::Address<Mcu::Components::Rcc>::value;
            if constexpr (TimerNumber == 1) {
                rcc->APBRSTR2 |= RCC_APBRSTR2_TIM1RST;
                rcc->APBRSTR2 &= ~RCC_APBRSTR2_TIM1RST;
            }
#ifdef RCC_APBRSTR1_TIM2RST
            else if constexpr (TimerNumber == 2) {
                RCC->APBRSTR1 |= RCC_APBRSTR1_TIM2RST;
                RCC->APBRSTR1 &= ~RCC_APBRSTR1_TIM2RST;
            }
#endif
            else if constexpr (TimerNumber == 3) {
                RCC->APBRSTR1 |= RCC_APBRSTR1_TIM3RST;
                RCC->APBRSTR1 &= ~RCC_APBRSTR1_TIM3RST;
            }
#ifdef RCC_APBRSTR1_TIM4RST
            else if constexpr (TimerNumber == 4) {
                RCC->APBRSTR1 |= RCC_APBRSTR1_TIM4RST;
                RCC->APBRSTR1 &= ~RCC_APBRSTR1_TIM4RST;
            }
#endif
#ifdef RCC_APBRSTR1_TIM6RST
            else if constexpr (TimerNumber == 6) {
                RCC->APBRSTR1 |= RCC_APBRSTR1_TIM6RST;
                RCC->APBRSTR1 &= ~RCC_APBRSTR1_TIM6RST;
            }
#endif
#ifdef RCC_APBRSTR1_TIM7RST
            else if constexpr (TimerNumber == 7) {
                RCC->APBRSTR1 |= RCC_APBRSTR1_TIM7RST;
                RCC->APBRSTR1 &= ~RCC_APBRSTR1_TIM7RST;
            }
#endif
#ifdef RCC_APBRSTR2_TIM14RST
            else if constexpr (TimerNumber == 14) {
                RCC->APBRSTR2 |= RCC_APBRSTR2_TIM14RST;
                RCC->APBRSTR2 &= ~RCC_APBRSTR2_TIM14RST;
            }
#endif
            else {
                static_assert(false);
            }
        }
#endif


#ifdef STM32G4
        template<uint8_t TimerNumber>
        void powerUp() {
            if constexpr (TimerNumber == 1) {
                RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
            }
            else if constexpr (TimerNumber == 2) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
            }
            else if constexpr (TimerNumber == 3) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
            }
#ifdef RCC_APB1ENR1_TIM4EN
            else if constexpr (TimerNumber == 4) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
            }
#endif
#ifdef RCC_APB1ENR1_TIM5EN
            else if constexpr(TimerNumber == 5) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
            }
#endif
            else if constexpr(TimerNumber == 6) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
            }
            else if constexpr(TimerNumber == 7) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
            }
            else {
                static_assert(false);
            }
        }
#endif
#ifdef STM32G0
        template<uint8_t TimerNumber>
        void powerUp() {
            if constexpr (TimerNumber == 1) {
                RCC->APBENR2 |= RCC_APBENR2_TIM1EN;
            }
#ifdef RCC_APBENR1_TIM2EN
            else if constexpr (TimerNumber == 2) {
                RCC->APBENR1 |= RCC_APBENR1_TIM2EN;
            }
#endif
            else if constexpr (TimerNumber == 3) {
                RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
            }
#ifdef RCC_APBENR1_TIM4EN
            else if constexpr (TimerNumber == 4) {
                RCC->APBENR1 |= RCC_APBENR1_TIM4EN;
            }
#endif
#ifdef RCC_APBENR1_TIM6EN
            else if constexpr(TimerNumber == 6) {
                RCC->APBENR1 |= RCC_APBENR1_TIM6EN;
            }
#endif
#ifdef RCC_APBENR1_TIM7EN
            else if constexpr(TimerNumber == 7) {
                RCC->APBENR1 |= RCC_APBENR1_TIM7EN;
            }
#endif
#ifdef RCC_APBENR2_TIM14EN
            else if constexpr(TimerNumber == 14) {
                RCC->APBENR2 |= RCC_APBENR2_TIM14EN;
            }
#endif
            else {
                static_assert(false);
            }
        }
#endif
    }

    template<bool V = true>
    struct Trigger : std::integral_constant<bool, V> {};
    
    template<uint8_t N, typename Period, typename Prescaler, typename Trigger = Trigger<false>, typename MCU = DefaultMcu>
    struct Timer;

//    template<uint8_t N, uint16_t Per, uint16_t Pre, typename TR, typename MCU>
//    requires (N == 1)
//    struct Timer<N, std::integral_constant<uint16_t, Per>, std::integral_constant<uint16_t, Pre>, TR, MCU> {
//        using mcu_t = MCU;
//        using number_t = std::integral_constant<uint8_t, N>;

//        static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Timer<N, std::integral_constant<uint16_t, Per>, std::integral_constant<uint16_t, Pre>, MCU>>::value);
               
//        static inline void init() {
//            if constexpr(N == 1) {
//                RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
//            }
//            else {
//                static_assert(false);
//            }
//            mcuTimer->PSC = Pre;
//            mcuTimer->ARR = Per;
//            mcuTimer->CCR1 = Per / 2;
//            mcuTimer->CCR2 = Per / 4;
//            mcuTimer->CCR3 = Per / 6;
//            MODIFY_REG(mcuTimer->CCMR1, (TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk | TIM_CCMR1_OC1PE_Msk | TIM_CCMR1_OC2PE_Msk), 
//                       (0b0110 << TIM_CCMR1_OC1M_Pos) | (0b0110 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE
//                       );
//            mcuTimer->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
//            mcuTimer->BDTR |= TIM_BDTR_MOE;
//            mcuTimer->EGR |= TIM_EGR_UG;
//            mcuTimer->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
//        }        
//        static inline void duty(const float d1, const float d2, const float d3) {
//            mcuTimer->CCR1 = std::min<uint16_t>((d1 * Per), Per);            
//            mcuTimer->CCR2 = std::min<uint16_t>((d2 * Per), Per);            
//            mcuTimer->CCR3 = std::min<uint16_t>((d3 * Per), Per);            
//        }
//        static inline void duty1(const float d) {
//            mcuTimer->CCR1 = std::min<uint16_t>((d * Per), Per);            
//        }
//        static inline void duty2(const float d) {
//            mcuTimer->CCR2 = std::min<uint16_t>((d * Per), Per);            
//        }
//        static inline void duty3(const float d) {
//            mcuTimer->CCR3 = std::min<uint16_t>((d * Per), Per);            
//        }
//    };
    
    template<uint8_t N, uint16_t Per, uint16_t Pre, bool Tr, typename MCU>
    requires (N >= 3) && (N <= 4)
    struct Timer<N, std::integral_constant<uint16_t, Per>, std::integral_constant<uint16_t, Pre>, Trigger<Tr>, MCU> {
        using mcu_t = MCU;
        using number_t = std::integral_constant<uint8_t, N>;
//        using component_t = Mcu::Components::Timer<N>;

        // static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Timer<N, std::integral_constant<uint16_t, Per>, std::integral_constant<uint16_t, Pre>, MCU>>::value);
        static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Timer<N>>::value);
        
#ifdef STM32G4
        static inline void init() {
            if constexpr(N == 1) {
                RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
            }
            else if constexpr(N == 3) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
            }
            else if constexpr(N == 4) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
            }
            else {
                static_assert(false);
            }
            mcuTimer->PSC = Pre;
            mcuTimer->ARR = Per;
            mcuTimer->CCR1 = Per / 4;
            mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC1M_Pos); // pwm1
            mcuTimer->CCMR1 |= TIM_CCMR1_OC1PE;
            mcuTimer->CCER |= TIM_CCER_CC1E;
            if constexpr(Tr) {
                mcuTimer->CR2 |= 0x02 << TIM_CR2_MMS_Pos; // trgo
            }
            mcuTimer->EGR |= TIM_EGR_UG;
            mcuTimer->CR1 |= TIM_CR1_ARPE;
            mcuTimer->CR1 |= TIM_CR1_CEN;
        }        
#endif
        static inline void duty(const uint16_t d) {
            mcuTimer->CCR1 = d;            
        }
    };

    template<uint8_t N, typename Period, typename Prescaler, typename MCU = DefaultMcu>
    struct Waiter;
    template<uint8_t N, uint16_t Per, uint16_t Pre, typename MCU>
    requires (N >= 6) && (N <= 7)
    struct Waiter<N, std::integral_constant<uint16_t, Per>, std::integral_constant<uint16_t, Pre>, MCU> {
        using mcu_t = MCU;
        using number_t = std::integral_constant<uint8_t, N>;

        // std::integral_constant<uint16_t, Pre>::_;

        static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Timer<N>>::value);

#ifdef STM32G4
        static inline void init() {
            if constexpr(N == 6) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
            }
            else if constexpr(N == 7) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
            }
            else {
                static_assert(false);
            }
            mcuTimer->PSC = Pre;
            mcuTimer->ARR = Per;
        }
#endif

        static inline void start() {
            mcuTimer->CNT = 0;
            mcuTimer->SR &= ~TIM_SR_UIF;
            // mcuTimer->EGR |= TIM_EGR_UG;
            // mcuTimer->CR1 |= TIM_CR1_ARPE;
            mcuTimer->CR1 |= TIM_CR1_OPM;
            mcuTimer->DIER |= TIM_DIER_UIE;
            mcuTimer->CR1 |= TIM_CR1_CEN;
        }
        static inline void stop() {
            mcuTimer->CR1 &= ~TIM_CR1_CEN;
            mcuTimer->SR &= ~TIM_SR_UIF;
            mcuTimer->DIER &= ~TIM_DIER_UIE;
        }
    };

    template<uint8_t N, uint16_t Per, uint16_t Pre, bool Tr, typename MCU>
    requires (N >= 6) && (N <= 7)
    struct Timer<N, std::integral_constant<uint16_t, Per>, std::integral_constant<uint16_t, Pre>, Trigger<Tr>, MCU> {
        using mcu_t = MCU;
        using number_t = std::integral_constant<uint8_t, N>;

        static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Timer<N, std::integral_constant<uint16_t, Per>, std::integral_constant<uint16_t, Pre>, MCU>>::value);

        static inline constexpr uint8_t trgo() { // adc trigger select
            if constexpr(N == 6) {
                return 13;
            }
            else if constexpr(N == 7) {
                return 30;
            }
        }
        
#ifdef STM32G4
        static inline void init() {
            if constexpr(N == 6) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
            }
            else if constexpr(N == 7) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
            }
            else {
                static_assert(false);
            }
            mcuTimer->PSC = Pre;
            mcuTimer->ARR = Per;
            mcuTimer->EGR |= TIM_EGR_UG;
            mcuTimer->CR1 |= TIM_CR1_ARPE;
            if constexpr(Tr) {
                mcuTimer->CR2 |= 0x02 << TIM_CR2_MMS_Pos; // trgo
            }
            mcuTimer->CR1 |= TIM_CR1_CEN;
        }
#endif
        static inline uint16_t value() {
            return mcuTimer->CNT;
        }
    };
}


namespace Mcu {
    namespace Stm {
        template<>
        struct Address<Mcu::Components::Timer<1>> {
            static inline constexpr uintptr_t value = TIM1_BASE;
        };
#ifdef TIM2_BASE
        template<>
        struct Address<Mcu::Components::Timer<2>> {
            static inline constexpr uintptr_t value = TIM2_BASE;
        };
#endif
        template<>
        struct Address<Mcu::Components::Timer<3>> {
            static inline constexpr uintptr_t value = TIM3_BASE;
        };
#ifdef TIM4_BASE
        template<>
        struct Address<Mcu::Components::Timer<4>> {
            static inline constexpr uintptr_t value = TIM4_BASE;
        };
#endif
#ifdef STM32G4
        template<>
        struct Address<Mcu::Components::Timer<6>> {
            static inline constexpr uintptr_t value = TIM6_BASE;
        };
#endif
#ifdef STM32G4
        template<>
        struct Address<Mcu::Components::Timer<7>> {
            static inline constexpr uintptr_t value = TIM7_BASE;
        };
#endif
#ifdef STM32G0
        template<>
        struct Address<Mcu::Components::Timer<14>> {
            static inline constexpr uintptr_t value = TIM14_BASE;
        };
#endif
        template<>
        struct Address<Mcu::Components::Timer<16>> {
            static inline constexpr uintptr_t value = TIM16_BASE;
        };
        template<>
        struct Address<Mcu::Components::Timer<17>> {
            static inline constexpr uintptr_t value = TIM17_BASE;
        };
    }
}
