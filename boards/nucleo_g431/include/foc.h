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
                std::array<float, rotLength> data{};
                for(uint16_t i = 0; i < rotLength; ++i) {
                    data[i] = (1.0 + std::sin((2 * std::numbers::pi * i) / rotLength)) / 2.0;
                }
                return data;
            }();
            static inline constexpr auto PhaseV = []{
                std::array<float, rotLength> data{};
                for(uint16_t i = 0; i < rotLength; ++i) {
                    data[i] = (1.0 + std::sin((2 * std::numbers::pi * (i - 2 * intervallLength)) / rotLength)) / 2.0;
                }
                return data;
            }();
            static inline constexpr auto PhaseW = []{
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

            using sine = Sinus<1024>;
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
                mcuTimer->ARR = 100; 
                mcuTimer->EGR |= TIM_EGR_UG;
                mcuTimer->CR1 |= TIM_CR1_ARPE;
                mcuTimer->CR1 |= TIM_CR1_CEN;
            }
            static inline void start() {
                mcuTimer->DIER |= TIM_DIER_UIE;
                mRotation = 0;
                mMechSection = 0;
                mMeasuring = true;
                Interrupt::measureState = Interrupt::State::Wait;
            }
            static inline void stop() {
                mcuTimer->DIER &= ~TIM_DIER_UIE;
                mMeasuring = false;
            }
            static inline void period(const uint16_t p) {
                mcuTimer->ARR = p; 
            }
            static inline uint16_t period() {
                return mcuTimer->ARR; 
            }

            struct Interrupt {
                static inline void hall(const uint8_t hall) {
                    const uint8_t state = Motor::Util<MCU>::hallToState(hall);
                    int8_t stateDiff = state - mHallState;
                    mHallState = state;
                    if (stateDiff <= -4) stateDiff += 6;
                    if (stateDiff >=  4) stateDiff -= 6;
                    mMechSection += stateDiff;
                    if (mMechSection >= mMechSectionMax) {
                        mMechSection -= mMechSectionMax;
                        ++mRotation;
                    }
                    else if (mMechSection < 0) {
                        mMechSection += mMechSectionMax;
                        ++mRotation;
                    }
                }
                enum class State {Wait, Record, Done};
                
                static inline void measure(const uint16_t lastHallSectorLength) {
                    static uint16_t m = 0;
                    if (mMeasuring) {
                        switch(mMeasureState) {
                        case State::Wait:
                            if (mRotation > mWaitRotationsBeforeMeasuring) {
                                if (mMechSection == 0) {
                                    m = 0;
                                    mHallSectionLengths[mMechSection].lastLength = lastHallSectorLength;
                                    mHallSectionLengths[mMechSection].sineIndex = mSineIndex;
                                    ++m;
                                    mMeasureState = State::Record;
                                }
                            }
                        break;
                        case State::Record:
    //                        const Motor::Sector sector{lastHallSectorLength, mSineIndex};
    //                        mHallSectionLengths[m++] = sector;
                            mHallSectionLengths[mMechSection].lastLength = lastHallSectorLength;
                            mHallSectionLengths[mMechSection].sineIndex = mSineIndex;
                            ++m;
                            if (m >= (6 * 7)) {
    //                        if (m >= mHallSectionLengths.size()) {
                                mMeasureState = State::Done;
                                mMeasuring = false;
                            }
                        break;
                        case State::Done:
                        break;
                        }
                    }
                }
                static inline void isr() {
                    mcuTimer->SR = ~TIM_SR_UIF; // clear if
                    const uint16_t i = mSineIndex;
                    driver::duty(sine::PhaseU[i], sine::PhaseV[i], sine::PhaseW[i]);    
                    ++mSineIndex;
                }
            private:
                static inline State mMeasureState{State::Wait};
                static inline volatile etl::ranged_circular<0, sine::length - 1> mSineIndex;
            public:
                static inline volatile const auto& measureState{mMeasureState};
            };

            static inline Motor::Sector actual() {
                Motor::Sector s;
                s.lastLength = mHallSectionLengths[mMechSection].lastLength;
                s.sineIndex = mHallSectionLengths[mMechSection].sineIndex;
                return s;
//                return mHallSectionLengths[mMechSection];
            }
            static inline uint16_t section() {
                return mMechSection;
            }
            
            static inline uint16_t mWaitRotationsBeforeMeasuring{3};
            static inline uint8_t mHallState{};
            static inline uint16_t mPolePairs{7};
            static inline volatile int16_t mMechSection{};
            static inline int16_t mMechSectionMax{6 * mPolePairs};
            static inline volatile uint32_t mRotation{};
            
            static inline volatile bool mMeasuring{};
//            static inline std::array<volatile Motor::Sector, 6 * 7> mHallSectionLengths{};
            static inline volatile Motor::Sector mHallSectionLengths[6 * 7];
        };
        
        
        template<uint8_t N, typename PreScaler, typename Driver, typename Measure, typename Dac, typename MCU>
        struct Estimator;
        
        template<uint8_t N, uint16_t Pre, typename Driver, typename Measure, typename Dac, typename MCU>
        requires ((N == 6) || (N == 7))
        struct Estimator<N, std::integral_constant<uint16_t, Pre>, Driver, Measure, Dac, MCU> {
            using dac = Dac;
            using driver = Driver;
            using measure = Measure;
            
            static inline constexpr uint16_t k = 3;
            
            static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Timer<N, void, void, MCU>>::value);
            
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
//                mcuTimer->ARR = Measure::period(); 
//                mcuTimer->ARR = 100; 
                mcuTimer->ARR = 100 / k; 
                mcuTimer->EGR |= TIM_EGR_UG;
                mcuTimer->CR1 |= TIM_CR1_ARPE;
                mcuTimer->CR1 |= TIM_CR1_CEN;
            }
            static inline void start() {
                mcuTimer->DIER |= TIM_DIER_UIE;
            }
            static inline void stop() {
                mcuTimer->DIER &= ~TIM_DIER_UIE;
            }

            struct Interrupt {
                static inline void update(const uint16_t lastHallSectorLength) {
                    const Motor::Sector actual = measure::actual();
                    const float v1 = (1.0 * actual.lastLength) / lastHallSectorLength;
                    const float v = mExpMean.process(v1);
                    const float si = actual.sineIndex + v + (mAngleFaktor * measure::sine::length) / 8;
                    mSineIndex = si;
                    mTickIncrement = v / k;
                }            
                static inline void isr() {
                    mcuTimer->SR = ~TIM_SR_UIF; // clear if
                    const uint16_t index = ((uint16_t)mSineIndex) % measure::sine::length;
                    driver::duty(measure::sine::PhaseU[index], measure::sine::PhaseV[index], measure::sine::PhaseW[index]);
                    mSineIndex += mTickIncrement;
                    dac::set2(4 * index);
                }
            private:
                static inline  Dsp::ExpMean<void> mExpMean{0.1};
                static inline volatile float mSineIndex = 0;
                static inline volatile float mTickIncrement = 0;
            public:
                static inline volatile const auto& expMean{mExpMean};
                static inline volatile float mAngleFaktor = 1.0;
            };
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
//                scale(0);
                both<PhaseU>();
                both<PhaseV>();
                both<PhaseW>();
            }
            static inline void all_off() {
                floating<PhaseU>();
                floating<PhaseV>();
                floating<PhaseW>();
//                scale(0);
            }
//        private:
            volatile static inline float mScale = 0.0;
        };
    }
    
    template<uint8_t N, typename Prescaler, typename Driver, typename E, typename Measure, typename MCU = DefaultMcu> struct Hall;

    template<uint8_t N, uint16_t Pre, typename Driver, typename PosEst, typename Measure, typename MCU>
    requires (N == 4)
    struct Hall<N, std::integral_constant<uint16_t, Pre>, Driver, PosEst, Measure, MCU> {
        using mcu_t = MCU;
        using number_t = std::integral_constant<uint8_t, N>;

        using estimator = PosEst;
        using measure = Measure;
        
        static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Timer<N, std::integral_constant<uint16_t, 0>, std::integral_constant<uint16_t, Pre>, MCU>>::value);

        struct Interrupt {
            static inline void isr() {
                mcuTimer->SR = ~TIM_SR_CC2IF; // reset if
                const uint8_t hall = ((GPIOB->IDR >> 6) & 0b0111);
                
                measure::Interrupt::hall(hall);
                
                if (!(mcuTimer->SR & TIM_SR_CC1OF)) {
                    const uint16_t lastHallSectorLength = mcuTimer->CCR1;
                    measure::Interrupt::measure(lastHallSectorLength);
                    estimator::Interrupt::update(lastHallSectorLength);
                }
                else {
                    mcuTimer->SR = ~TIM_SR_CC1OF;
                    ++mOverTrigger;
                }
            }
        private:
            static inline uint16_t mOverTrigger{};
        public:
            volatile static inline const auto& overTrigger{mOverTrigger};
        };
        
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
    };    
    
}
