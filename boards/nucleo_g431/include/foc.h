#pragma once

#include "mcu.h"
#include "timer.h"
#include "units.h"
#include "concepts.h"
#include "mcu_traits.h"
#include "meta.h"
#include "alternate.h"
#include "ranged.h"
#include "dsp.h"

#include <type_traits>
#include <concepts>
#include <algorithm>
#include <array>
#include <numeric>
#include <numbers>

namespace Mcu::Stm {
    using namespace Units::literals;
    namespace Motor {
        template<typename MCU = DefaultMcu>
        struct Util {
            static inline constexpr std::array<uint8_t, 6> hallValues {
                0b00000001,
                0b00000101,
                0b00000100,
                0b00000110,
                0b00000010,
                0b00000011,
            };        
            
            static inline constexpr auto hallValuesToStepLut = []{
                std::array<uint8_t, 8> values {};
                for(uint8_t s{0}; const auto& hv: hallValues) {
                    values[hv] = s++;
                }    
                return values;
            }();
            
            static inline constexpr auto hallToState(const uint8_t h) {
                return hallValuesToStepLut[h & 0x07];
            }
        };
        
        template<uint16_t rotLength>
        struct Sinus {
            static inline constexpr uint16_t length = rotLength;
            static inline constexpr uint16_t intervallLength = rotLength / 6;
            static inline constexpr auto PhaseU = []{
                const uint16_t start = 0;
                std::array<float, rotLength> data{};
                for(uint16_t i = 0; i < rotLength; ++i) {
                    data[i] = (1.0 + std::sin((2 * std::numbers::pi * i) / rotLength)) / 2.0;
                }
                return data;
            }();
            static inline constexpr auto PhaseV = []{
                const uint16_t start = 0;
                std::array<float, rotLength> data{};
                for(uint16_t i = 0; i < rotLength; ++i) {
                    data[i] = (1.0 + std::sin((2 * std::numbers::pi * (i - 2 * intervallLength)) / rotLength)) / 2.0;
                }
                return data;
            }();
            static inline constexpr auto PhaseW = []{
                const uint16_t start = 0;
                std::array<float, rotLength> data{};
                for(uint16_t i = 0; i < rotLength; ++i) {
                    data[i] = (1.0 + std::sin((2 * std::numbers::pi * (i - 4 * intervallLength)) / rotLength)) / 2.0;
                }
                return data;
            }();
            
        };

        struct Sector {
            uint16_t lastLength;
            uint16_t sineIndex;
        };
        
        template<uint8_t N, typename Pre, typename Driver, typename MCU = DefaultMcu> struct Measurement;
        template<uint8_t N, uint16_t Pre, typename Driver, typename MCU>
        requires (N >= 6) && (N <= 7)
        struct Measurement<N, std::integral_constant<uint16_t, Pre>, Driver, MCU> {
            using mcu_t = MCU;
            using number_t = std::integral_constant<uint8_t, N>;
    
            static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Timer<N, void, void, MCU>>::value);

            using sine = Sinus<128>;
            using driver = Driver;
            
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
                mcuTimer->ARR = 1000; 
                mcuTimer->DIER |= TIM_DIER_UIE;
                mcuTimer->EGR |= TIM_EGR_UG;
                mcuTimer->CR1 |= TIM_CR1_ARPE;
                mcuTimer->CR1 |= TIM_CR1_CEN;
            }
        
            static inline void period(const uint16_t p) {
                mcuTimer->ARR = p; 
            }
            
            static inline uint16_t index() {
                return mIndex;
            }
            static inline void isr() {
                mcuTimer->SR = ~TIM_SR_UIF; // clear if
                driver::duty(sine::PhaseU[mIndex], sine::PhaseV[mIndex], sine::PhaseW[mIndex]);    
                ++mIndex;
            }
            private:
            static inline etl::ranged_circular<0, sine::length - 1> mIndex;
        };
        
        
        template<uint8_t N, typename PreScaler, typename Driver, typename Dac, typename MCU>
        struct Estimator;
        
        template<uint8_t N, uint16_t Pre, typename Driver, typename Dac, typename MCU>
        requires ((N == 6) || (N == 7))
        struct Estimator<N, std::integral_constant<uint16_t, Pre>, Driver, Dac, MCU> {
            using dac = Dac;
            using driver = Driver;
            
            static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Timer<N, void, void, MCU>>::value);
            
            static inline constexpr uint16_t intervallLength = 64;
            static inline constexpr uint16_t rotLength = 6 * intervallLength;
//            static inline constexpr uint16_t time = 100; // 100 µs 
            static inline constexpr uint16_t time = 50; // 100 µs 

            using sine = Sinus<rotLength>;
                       
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
                mcuTimer->ARR = time; 
                mcuTimer->DIER |= TIM_DIER_UIE;
                mcuTimer->EGR |= TIM_EGR_UG;
                mcuTimer->CR1 |= TIM_CR1_ARPE;
                mcuTimer->CR1 |= TIM_CR1_CEN;
            }
            static inline void set(const Sector sector) {
            }            
//            static inline void set(const uint8_t s, const float intTimeEst) {
//                const uint16_t deg90 = rotLength / 8;
////                const uint16_t deg90 = 0;
////                const uint8_t state = (s + 3) % 6; // sin: 2 // saddle: 3
//                const uint8_t state = (s + 2) % 6; // sin: 2 // saddle: 3
//                if (state == 0) {
//                    mPos = deg90;
//                }
//                mTickIncrement = (1.0f * intervallLength * time) / std::max(1.0f, intTimeEst);
//            }
            static inline void isr() {
                mcuTimer->SR = ~TIM_SR_UIF; // clear if
//                mPos += mTickIncrement;
//                const uint16_t index = (uint16_t)mPos % rotLength;
//                driver::duty(sine::PhaseU[index], sine::PhaseV[index], sine::PhaseW[index]);
            }
//        private:
            static inline float mPos = 0;
            static inline volatile float mTickIncrement = 0;
        public:
            static inline volatile const auto& pos{mPos};
        };
        
        template<typename Pin, typename Function>
        struct Switch {
            using pin = Pin;
            using function = Function;
        };
        
        template<typename HighSide, typename LowSide>
        struct Phase {
            using highSide = HighSide;  
            using lowSide  = LowSide;  
        };
        
        template<uint8_t N, typename PhaseU, typename PhaseV, typename PhaseW, typename Period, typename Clock, typename MCU = DefaultMcu> 
        struct Driver;
    
        template<uint8_t N, typename PhaseU, typename PhaseV, typename PhaseW, uint16_t Per, typename Clock, typename MCU>
        requires (N == 1)
        struct Driver<N, PhaseU, PhaseV, PhaseW, std::integral_constant<uint16_t, Per>, Clock, MCU> {
            using mcu_t = MCU;
            using number_t = std::integral_constant<uint8_t, N>;
            using component_t = Mcu::Components::Timer<N>;
            using this_t = Driver;
            
            static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Timer<N, void, void, MCU>>::value);
            
            static inline uint8_t trgo() {
                return 9;
            }
            static inline uint8_t trgo2() {
                return 10;
            }
            
            static inline void init() {
                if constexpr(N == 1) {
                    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
                }
                else {
                    static_assert(false);
                }
                mcuTimer->PSC = 0;
                mcuTimer->ARR = Per;
                mcuTimer->CCR1 = 0;
                mcuTimer->CCR2 = 0;
                mcuTimer->CCR3 = 0;
                mcuTimer->CCR4 = 1;
                
                MODIFY_REG(mcuTimer->CCMR1, (TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk | TIM_CCMR1_OC1PE_Msk | TIM_CCMR1_OC2PE_Msk), 
                           (0b0110 << TIM_CCMR1_OC1M_Pos) | (0b0110 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE
                           );
                MODIFY_REG(mcuTimer->CCMR2, (TIM_CCMR2_OC3M_Msk | TIM_CCMR2_OC4M_Msk | TIM_CCMR2_OC3PE_Msk | TIM_CCMR2_OC4PE_Msk), 
                           (0b0110 << TIM_CCMR2_OC3M_Pos) | (0b0110 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC3PE 
                           );
                mcuTimer->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
                mcuTimer->BDTR |= TIM_BDTR_MOE;
                mcuTimer->EGR |= TIM_EGR_UG;
                MODIFY_REG(mcuTimer->CR2, TIM_CR2_MMS_Msk, 0b0111 << TIM_CR2_MMS_Pos); // oc4 -> tim1_trgo
                MODIFY_REG(mcuTimer->CR1, TIM_CR1_CMS_Msk, 0b01 << TIM_CR1_CMS_Pos);
                mcuTimer->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
                
                floating<PhaseU>();
                floating<PhaseV>();
                floating<PhaseW>();
            }
            static inline void duty(const float u, const float v, const float w) {
                mcuTimer->CCR1 = std::min<uint16_t>((u * mScale * Per), Per);            
                mcuTimer->CCR2 = std::min<uint16_t>((v * mScale * Per), Per);            
                mcuTimer->CCR3 = std::min<uint16_t>((w * mScale * Per), Per);            
            }
            static inline void scale(const float d) {
                mScale = d;
            }
            template<typename Phase>
            static inline void floating() {
                Phase::highSide::pin::template dir<Output>();                
                Phase::highSide::pin::reset();
                Phase::lowSide::pin::template dir<Output>();                
                Phase::lowSide::pin::reset();
            }
            template<typename Phase>
            static inline void low() {
                using hs_t = Phase::highSide::pin;
                hs_t::template dir<Output>();                
                hs_t::reset();
                using ls_t = Phase::lowSide::pin;
                ls_t::template dir<Output>();                
                ls_t::set();
            }
            template<typename Phase>
            static inline void pwm() {
                using hs_t = Phase::highSide::pin;
                using hf_t = Phase::highSide::function;
                hs_t::afunction(AlternateFunctions::mapper_v<hs_t, this_t, hf_t>); 
                using ls_t = Phase::lowSide::pin;
                using lf_t = Phase::lowSide::function;
                ls_t::afunction(AlternateFunctions::mapper_v<ls_t, this_t, lf_t>); 
            }
            template<typename Phase>
            static inline void both() {
                using hp_t = Phase::highSide::pin;
                using hf_t = Phase::highSide::function;
                hp_t::afunction(AlternateFunctions::mapper_v<hp_t, this_t, hf_t>); 
                using lp_t = Phase::lowSide::pin;
                using lf_t = Phase::lowSide::function;
                lp_t::afunction(AlternateFunctions::mapper_v<lp_t, this_t, lf_t>); 
            }
            static inline void all_on() {
                both<PhaseU>();
                both<PhaseV>();
                both<PhaseW>();
                scale(0);
            }
            static inline void all_off() {
                floating<PhaseU>();
                floating<PhaseV>();
                floating<PhaseW>();
                scale(0);
            }
        private:
            volatile static inline float mScale = 0.1;
        };
    }
    
    template<uint8_t N, typename Prescaler, typename Driver, typename E, typename Measure, typename MCU = DefaultMcu> struct Hall;

    template<uint8_t N, uint16_t Pre, typename Driver, typename PosEst, typename Measure, typename MCU>
    requires (N == 4)
    struct Hall<N, std::integral_constant<uint16_t, Pre>, Driver, PosEst, Measure, MCU> {
        using mcu_t = MCU;
        using number_t = std::integral_constant<uint8_t, N>;

        using pos = PosEst;
        using measure = Measure;
        
        static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Timer<N, std::integral_constant<uint16_t, 0>, std::integral_constant<uint16_t, Pre>, MCU>>::value);

        static inline void isr() {
            mcuTimer->SR = ~TIM_SR_CC2IF; // reset if
            const uint8_t hall = ((GPIOB->IDR >> 6) & 0b0111);
            const uint8_t state = Motor::Util<MCU>::hallToState(hall);
            int8_t stateDiff = state - mState;
            mState = state;
            if (stateDiff == -5) stateDiff = 1;
            if (stateDiff ==  5) stateDiff = -1;
            mMechSection += stateDiff;
            if (mMechSection >= mMechSectionMax) {
                mMechSection -= mMechSectionMax;
                ++mRotations;
            }
            if (mMechSection < 0) {
                mMechSection += mMechSectionMax;
                ++mRotations;
            }
            if (mRotations >= mMechSectionLengths.size()) {
                mRotations = 0;
            }
            if (!(mcuTimer->SR & TIM_SR_CC1OF)) {
                Motor::Sector sector;
                sector.lastLength = mcuTimer->CCR1; // reset if
                sector.sineIndex = measure::index();
                mMechSectionLengths[mRotations][mMechSection] = sector;
                pos::set(sector);
            }
            else {
                mcuTimer->SR = ~TIM_SR_CC1OF;
                ++mOverTrigger;
            }
        }
        
        static inline void init() {
            if constexpr(N == 4) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
            }
            else {
                static_assert(false);
            }
            
            mcuTimer->PSC = Pre;

            MODIFY_REG(mcuTimer->TISEL, TIM_TISEL_TI1SEL_Msk, 0b0000 << TIM_TISEL_TI1SEL_Pos);
            MODIFY_REG(mcuTimer->TISEL, TIM_TISEL_TI2SEL_Msk, 0b0000 << TIM_TISEL_TI2SEL_Pos);
            MODIFY_REG(mcuTimer->TISEL, TIM_TISEL_TI3SEL_Msk, 0b0000 << TIM_TISEL_TI3SEL_Pos);
            
            MODIFY_REG(mcuTimer->CCMR1, TIM_CCMR1_CC1S_Msk, 0b01 << TIM_CCMR1_CC1S_Pos);
            MODIFY_REG(mcuTimer->CCMR1, TIM_CCMR1_IC1F_Msk, 0b1111 << TIM_CCMR1_IC1F_Pos); // Filter
            
            mcuTimer->CCER |= TIM_CCER_CC1P;
            mcuTimer->CCER |= TIM_CCER_CC1NP;
            mcuTimer->CCER |= TIM_CCER_CC1E;
            
            mcuTimer->DIER |= TIM_DIER_CC1IE;

//            mcuTimer->DIER |= TIM_DIER_CC2IE;
            
//            mcuTimer->CCR2 = 1; // delay
//            MODIFY_REG(mcuTimer->CCMR1, TIM_CCMR1_OC2M_Msk, 0b0111 << TIM_CCMR1_OC2M_Pos);
//            MODIFY_REG(mcuTimer->CR2, TIM_CR2_MMS_Msk, 0b0101 << TIM_CR2_MMS_Pos);
//            mcuTimer->CCER |= TIM_CCER_CC2P;
                    
            mcuTimer->CR2 |= TIM_CR2_TI1S;
            MODIFY_REG(mcuTimer->SMCR, TIM_SMCR_SMS_Msk, 0b0100 << TIM_SMCR_SMS_Pos);
            MODIFY_REG(mcuTimer->SMCR, TIM_SMCR_TS_Msk, 0b0100 << TIM_SMCR_TS_Pos);
            
            mcuTimer->EGR |= TIM_EGR_UG;
            mcuTimer->CR1 |= TIM_CR1_CEN;

//            mHalls = ((GPIOB->IDR >> 6) & 0b0111);
        }
    private:
        static inline uint8_t mState{};
        static inline uint16_t mOverTrigger{};
        static inline uint16_t mPolePairs{7};
        static inline int16_t mMechSection{};
        static inline int16_t mMechSectionMax{6 * mPolePairs};
        static inline uint8_t mRotations{};
        static inline std::array<std::array<Motor::Sector, 6 * 7>, 10> mMechSectionLengths {}; // 32 Poles
    public:
        volatile static inline const auto& state{mState};
        volatile static inline const auto& mechSection{mMechSection};
        /*volatile*/ static inline const auto& mechSectionLengths{mMechSectionLengths};
        volatile static inline const auto& overTrigger{mOverTrigger};
    };    
    
}
