/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include "timer.h"
#include "units.h"
#include "concepts.h"
#include "mcu/mcu_traits.h"
#include "meta.h"
#include "mcu/alternate.h"
#include "etl/ranged.h"
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
        namespace Identification {

            template<uint8_t PwmTimerNumber, uint8_t AdcTimerNumber,
                     typename PinLS1, typename PinHS1, typename PinLS2, typename PinHS2, typename Clock, typename MCU = DefaultMcu>
            struct BdcWithPins4 {
                using pwmTimer_ct = Mcu::Components::Timer<PwmTimerNumber>;
                using adcTimer_ct = Mcu::Components::Timer<AdcTimerNumber>;
                static inline /*constexpr */ TIM_TypeDef* const pwmTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<pwmTimer_ct>::value);
                static inline /*constexpr */ TIM_TypeDef* const adcTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<adcTimer_ct>::value);

                static_assert((PwmTimerNumber >= 2) && (PwmTimerNumber <= 5));
                static_assert((AdcTimerNumber >= 2) && (AdcTimerNumber <= 5));

                using pinls1 = PinLS1;
                using pinhs1 = PinHS1;
                using pinls2 = PinLS2;
                using pinhs2 = PinHS2;

                struct pwmTimer_t {
                    using component_t = pwmTimer_ct;
                };

                static inline constexpr uint8_t pinls1AF = Mcu::Stm::AlternateFunctions::mapper_v<pinls1, pwmTimer_t, Mcu::Stm::AlternateFunctions::CC<1>>;
                // std::integral_constant<uint8_t, pinls1AF>::_;
                static inline constexpr uint8_t pinhs1AF = Mcu::Stm::AlternateFunctions::mapper_v<pinhs1, pwmTimer_t, Mcu::Stm::AlternateFunctions::CC<2>>;
                // std::integral_constant<uint8_t, pinhs1AF>::_;
                static inline constexpr uint8_t pinls2AF = Mcu::Stm::AlternateFunctions::mapper_v<pinls2, pwmTimer_t, Mcu::Stm::AlternateFunctions::CC<3>>;
                // std::integral_constant<uint8_t, pinls2AF>::_;
                static inline constexpr uint8_t pinhs2AF = Mcu::Stm::AlternateFunctions::mapper_v<pinhs2, pwmTimer_t, Mcu::Stm::AlternateFunctions::CC<4>>;
                // std::integral_constant<uint8_t, pinhs2AF>::_;

                static inline void init() {
                    Mcu::Stm::Timers::powerUp<PwmTimerNumber>();
                    Mcu::Stm::Timers::powerUp<AdcTimerNumber>();

                    Mcu::Stm::Timers::reset<PwmTimerNumber>();
                    Mcu::Stm::Timers::reset<AdcTimerNumber>();

                    pwmTimer->PSC = prescaler;
                    pwmTimer->ARR = period;
                    pwmTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC1M_Pos); // pwm1
                    pwmTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC2M_Pos); // pwm2
                    pwmTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC3M_Pos); // pwm3
                    pwmTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC4M_Pos); // pwm4
                    pwmTimer->CCER |= TIM_CCER_CC1E;
                    pwmTimer->CCER |= TIM_CCER_CC2E;
                    pwmTimer->CCER |= TIM_CCER_CC1P;
                    pwmTimer->CCER |= TIM_CCER_CC3E;
                    pwmTimer->CCER |= TIM_CCER_CC4E;
                    pwmTimer->CCER |= TIM_CCER_CC3P;
                    pwmTimer->CCR1 = 0;
                    pwmTimer->CCR2 = 0;
                    pwmTimer->CCR3 = 0;
                    pwmTimer->CCR4 = 0;
                    pwmTimer->CR1 |= TIM_CR1_ARPE;
                    pwmTimer->CR2 |= (0b0010 << TIM_CR2_MMS_Pos); // update as trigger-output

                    pwmTimer->DIER |= TIM_DIER_UIE;
                    pwmTimer->DIER |= TIM_DIER_CC1IE;

                    pwmTimer->CR1 |= TIM_CR1_CEN;
                }

                // Achtung: dmaStorage muss groß genug sein
                static inline void setMultiMode() {
                    Mcu::Stm::Timers::reset<AdcTimerNumber>();
                    adcTimer->PSC = prescaler;
                    adcTimer->ARR = period / 100;
                    // adcTimer->ARR = period / 200;

                    adcTimer->SMCR |= (0b001 << TIM_SMCR_SMS_Pos); // gated mode + reset
                    adcTimer->SMCR |= TIM_SMCR_SMS_3;

                    // pwmTimer: cc1 / cc2 as trgo
                    MODIFY_REG(pwmTimer->CR2, TIM_CR2_MMS_Msk, (0b0100 << TIM_CR2_MMS_Pos));

                    adcTimer->SMCR |= (Timers::trgoToTrigger<PwmTimerNumber, AdcTimerNumber>() << TIM_SMCR_TS_Pos); //TIM3!!!!
                    adcTimer->CR1 |= TIM_CR1_ARPE;
                    adcTimer->CR2 |= (0b0010 << TIM_CR2_MMS_Pos); // update as trgo

                    // adcTimer->DIER |= TIM_DIER_UIE;

                    adcTimer->CR1 |= TIM_CR1_CEN;
                }
                static inline void setSingleMode() {
                    Mcu::Stm::Timers::reset<AdcTimerNumber>();
                    adcTimer->PSC = prescaler;
                    adcTimer->ARR = period;

                    adcTimer->SMCR |= (0b000 << TIM_SMCR_SMS_Pos);
                    adcTimer->SMCR |= TIM_SMCR_SMS_3;

                    MODIFY_REG(pwmTimer->CR2, TIM_CR2_MMS_Msk, (0b0010 << TIM_CR2_MMS_Pos));// update as trigger-output

                    adcTimer->SMCR |= (Timers::trgoToTrigger<PwmTimerNumber, AdcTimerNumber>() << TIM_SMCR_TS_Pos); //TIM3!!!!

                    adcTimer->CCMR1 |= (0b1000 << TIM_CCMR1_OC1M_Pos); // retriggerable OPM
                    adcTimer->CR1 |= TIM_CR1_ARPE;
                    // adcTimer->CR2 |= (0b0100 << TIM_CR2_MMS_Pos); // cc1 as trgo
                    adcTimer->CR2 |= (0b0011 << TIM_CR2_MMS_Pos); // CC1IF trgo

                    adcTimer->DIER |= TIM_DIER_CC1IE; // test only

                    adcTimer->CR1 |= TIM_CR1_CEN;
                }
                static inline void setToneMode() {
                    Mcu::Stm::Timers::reset<AdcTimerNumber>();

                    // pwmTimer: cc1 / cc2 as trgo
                    MODIFY_REG(pwmTimer->CR2, TIM_CR2_MMS_Msk, (0b0100 << TIM_CR2_MMS_Pos));
                }

                static inline uint16_t pwmFreq() {
                    return freq;
                }
                static inline uint16_t maxDuty() {
                    return period;
                }
                static inline void pwm(const uint16_t f) {
                    freq = f;
                    prescaler = (Clock::config::frequency.value / (freq * period));
                    pwmTimer->PSC = prescaler;
                    pwmTimer->EGR |= TIM_EGR_UG;
                    adcTimer->PSC = prescaler;
                    adcTimer->EGR |= TIM_EGR_UG;
                }
                static inline void duty(const uint16_t v) {
                    if (v <= period) {
                        pwmTimer->CCR1 = v;
                        pwmTimer->CCR2 = v;
                        pwmTimer->CCR3 = v;
                        pwmTimer->CCR4 = v;
                        adcTimer->CCR1 = std::max(1.0f, (triggerTiming * v)); // trigger
                    }
                }
                template<template<auto, auto> typename T, auto L, auto U>
                static inline void duty(const T<L, U>& v) {
                    if (v) {
                        const uint16_t vv = period * ((float)v.toInt() - L) / (U - L);
                        pwmTimer->CCR1 = vv;
                        pwmTimer->CCR2 = vv;
                        pwmTimer->CCR3 = vv;
                        pwmTimer->CCR4 = vv;
                        adcTimer->CCR1 = std::max(1.0f, (triggerTiming * vv)); // trigger
                    }
                }

                static inline void dir1() {
                    pinls1::set();
                    pinhs1::reset();
                    pinls1::template dir<Mcu::Output>();
                    pinhs1::template dir<Mcu::Output>();
                    pinls2::afunction(pinls2AF);
                    pinhs2::afunction(pinhs2AF);
                }
                static inline void dir2() {
                    pinls2::set();
                    pinhs2::reset();
                    pinls2::template dir<Mcu::Output>();
                    pinhs2::template dir<Mcu::Output>();
                    pinls1::afunction(pinls1AF);
                    pinhs1::afunction(pinhs1AF);
                }
                static inline void trigger(const float v) {
                    triggerTiming = v;
                }
                constexpr static inline uint8_t trgo() {
                    if constexpr(AdcTimerNumber == 3) {
                        return 4; // tim3-trgo
                    }
                    else if constexpr(AdcTimerNumber == 2) {
                        return 11; // tim2-trgo
                    }
                    else if constexpr(AdcTimerNumber == 4) {
                        return 12; // tim4-trgo
                    }
                }
                private:
                static inline uint32_t period = 1640;
                static inline uint32_t freq   = 24000;
                static inline uint16_t prescaler = (Clock::config::frequency.value / (freq * period)) - 1;
                static inline float triggerTiming = 0.9f;
            };

            template<uint8_t PwmTimerNumber, uint8_t AdcTimerNumber, typename Pin1, typename Pin2, typename Clock, typename MCU = DefaultMcu>
            struct BdcWithPins {
                using pwmTimer_ct = Mcu::Components::Timer<PwmTimerNumber>;
                using adcTimer_ct = Mcu::Components::Timer<AdcTimerNumber>;
                static inline /*constexpr */ TIM_TypeDef* const pwmTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<pwmTimer_ct>::value);
                static inline /*constexpr */ TIM_TypeDef* const adcTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<adcTimer_ct>::value);

                static_assert((PwmTimerNumber >= 2) && (PwmTimerNumber <= 5));
                static_assert((AdcTimerNumber >= 2) && (AdcTimerNumber <= 5));

                using pin1 = Pin1;
                using pin2 = Pin2;

                struct pwmTimer_t {
                    using component_t = pwmTimer_ct;
                };

                static inline constexpr uint8_t pin1AF = Mcu::Stm::AlternateFunctions::mapper_v<pin1, pwmTimer_t, Mcu::Stm::AlternateFunctions::CC<1>>;
                // std::integral_constant<uint8_t, pin1AF>::_;
                static inline constexpr uint8_t pin2AF = Mcu::Stm::AlternateFunctions::mapper_v<pin2, pwmTimer_t, Mcu::Stm::AlternateFunctions::CC<2>>;
                // std::integral_constant<uint8_t, pin2AF>::_;

                static inline void init() {
                    Mcu::Stm::Timers::powerUp<PwmTimerNumber>();
                    Mcu::Stm::Timers::powerUp<AdcTimerNumber>();

                    Mcu::Stm::Timers::reset<PwmTimerNumber>();
                    Mcu::Stm::Timers::reset<AdcTimerNumber>();

                    pwmTimer->PSC = prescaler;
                    pwmTimer->ARR = period;
                    pwmTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC1M_Pos); // pwm1
                    pwmTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC2M_Pos); // pwm2
                    pwmTimer->CCER |= TIM_CCER_CC1E;
                    pwmTimer->CCER |= TIM_CCER_CC2E;
                    pwmTimer->CCR1 = 0;
                    pwmTimer->CCR2 = 0;
                    pwmTimer->CR1 |= TIM_CR1_ARPE;
                    pwmTimer->CR2 |= (0b0010 << TIM_CR2_MMS_Pos); // update as trigger-output

                    pwmTimer->DIER |= TIM_DIER_UIE;
                    pwmTimer->DIER |= TIM_DIER_CC1IE;

                    pwmTimer->CR1 |= TIM_CR1_CEN;
                }

                // Achtung: dmaStorage muss groß genug sein
                static inline void setMultiMode() {
                    Mcu::Stm::Timers::reset<AdcTimerNumber>();
                    adcTimer->PSC = prescaler;
                    adcTimer->ARR = period / 100;
                    // adcTimer->ARR = period / 200;

                    adcTimer->SMCR |= (0b001 << TIM_SMCR_SMS_Pos); // gated mode + reset
                    adcTimer->SMCR |= TIM_SMCR_SMS_3;

                    // pwmTimer: cc1 / cc2 as trgo
                    MODIFY_REG(pwmTimer->CR2, TIM_CR2_MMS_Msk, (0b0100 << TIM_CR2_MMS_Pos));

                    adcTimer->SMCR |= (Timers::trgoToTrigger<PwmTimerNumber, AdcTimerNumber>() << TIM_SMCR_TS_Pos); //TIM3!!!!
                    adcTimer->CR1 |= TIM_CR1_ARPE;
                    adcTimer->CR2 |= (0b0010 << TIM_CR2_MMS_Pos); // update as trgo

                    // adcTimer->DIER |= TIM_DIER_UIE;

                    adcTimer->CR1 |= TIM_CR1_CEN;
                }
                static inline void setSingleMode() {
                    Mcu::Stm::Timers::reset<AdcTimerNumber>();
                    adcTimer->PSC = prescaler;
                    adcTimer->ARR = period;

                    adcTimer->SMCR |= (0b000 << TIM_SMCR_SMS_Pos);
                    adcTimer->SMCR |= TIM_SMCR_SMS_3;

                    MODIFY_REG(pwmTimer->CR2, TIM_CR2_MMS_Msk, (0b0010 << TIM_CR2_MMS_Pos));// update as trigger-output

                    adcTimer->SMCR |= (Timers::trgoToTrigger<PwmTimerNumber, AdcTimerNumber>() << TIM_SMCR_TS_Pos); //TIM3!!!!

                    adcTimer->CCMR1 |= (0b1000 << TIM_CCMR1_OC1M_Pos); // retriggerable OPM
                    adcTimer->CR1 |= TIM_CR1_ARPE;
                    // adcTimer->CR2 |= (0b0100 << TIM_CR2_MMS_Pos); // cc1 as trgo
                    adcTimer->CR2 |= (0b0011 << TIM_CR2_MMS_Pos); // CC1IF trgo

                    // adcTimer->DIER |= TIM_DIER_CC1IE; // test only

                    adcTimer->CR1 |= TIM_CR1_CEN;
                }
                static inline void setToneMode() {
                    Mcu::Stm::Timers::reset<AdcTimerNumber>();

                    // pwmTimer: cc1 / cc2 as trgo
                    MODIFY_REG(pwmTimer->CR2, TIM_CR2_MMS_Msk, (0b0100 << TIM_CR2_MMS_Pos));
                }

                static inline uint16_t pwmFreq() {
                    return freq;
                }
                static inline uint16_t maxDuty() {
                    return period;
                }
                static inline void pwm(const uint16_t f) {
                    freq = f;
                    prescaler = (Clock::config::frequency.value / (freq * period));
                    pwmTimer->PSC = prescaler;
                    pwmTimer->EGR |= TIM_EGR_UG;
                    adcTimer->PSC = prescaler;
                    adcTimer->EGR |= TIM_EGR_UG;
                }
                static inline void duty(const uint16_t v) {
                    if (v <= period) {
                        pwmTimer->CCR1 = v;
                        pwmTimer->CCR2 = v;
                        adcTimer->CCR1 = std::max(1.0f, (triggerTiming * v)); // trigger
                    }
                }
                template<template<auto, auto> typename T, auto L, auto U>
                static inline void duty(const T<L, U>& v) {
                    if (v) {
                        const uint16_t vv = period * ((float)v.toInt() - L) / (U - L);
                        pwmTimer->CCR1 = vv;
                        pwmTimer->CCR2 = vv;
                        adcTimer->CCR1 = std::max(1.0f, (triggerTiming * vv)); // trigger
                    }
                }

                static inline void dir1() {
                    pin1::template dir<Mcu::Output>();
                    pin2::afunction(pin2AF);
                }
                static inline void dir2() {
                    pin2::template dir<Mcu::Output>();
                    pin1::afunction(pin1AF);
                }

                static inline void trigger(const float v) {
                }

                constexpr static inline uint8_t trgo() {
                    if constexpr(AdcTimerNumber == 3) {
                        return 4; // tim3-trgo
                    }
                    else if constexpr(AdcTimerNumber == 2) {
                        return 11; // tim2-trgo
                    }
                    else if constexpr(AdcTimerNumber == 4) {
                        return 12; // tim4-trgo
                    }
                }
                private:
                static inline uint32_t period = 1640;
                static inline uint32_t freq   = 24000;
                static inline uint16_t prescaler = (Clock::config::frequency.value / (freq * period)) - 1;
                static inline float triggerTiming = 0.9f;
            };

            template<uint8_t PwmTimerNumber, uint8_t AdcTimerNumber, typename Clock, typename MCU = DefaultMcu>
            struct Bdc {
                static inline /*constexpr */ TIM_TypeDef* const pwmTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Timer<PwmTimerNumber>>::value);
                static inline /*constexpr */ TIM_TypeDef* const adcTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Timer<AdcTimerNumber>>::value);

                static_assert((PwmTimerNumber >= 2) && (PwmTimerNumber <= 5));
                static_assert((AdcTimerNumber >= 2) && (AdcTimerNumber <= 5));

                // static inline constexpr uint8_t pwmAF = Mcu::Stm::AlternateFunctions::mapper_v<>;

                static inline void init() {
                    Mcu::Stm::Timers::powerUp<PwmTimerNumber>();
                    Mcu::Stm::Timers::powerUp<AdcTimerNumber>();

                    Mcu::Stm::Timers::reset<PwmTimerNumber>();
                    Mcu::Stm::Timers::reset<AdcTimerNumber>();

                    pwmTimer->PSC = prescaler;
                    pwmTimer->ARR = period;
                    pwmTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC1M_Pos); // pwm1
                    pwmTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC2M_Pos); // pwm2
                    pwmTimer->CCER |= TIM_CCER_CC1E;
                    pwmTimer->CCER |= TIM_CCER_CC2E;
                    pwmTimer->CCR1 = 0;
                    pwmTimer->CCR2 = 0;
                    pwmTimer->CR1 |= TIM_CR1_ARPE;
                    pwmTimer->CR2 |= (0b0010 << TIM_CR2_MMS_Pos); // update as trigger-output

                    pwmTimer->DIER |= TIM_DIER_UIE;
                    pwmTimer->DIER |= TIM_DIER_CC1IE;

                    pwmTimer->CR1 |= TIM_CR1_CEN;
                }

                // Achtung: dmaStorage muss groß genug sein
                static inline void setMultiMode() {
                    Mcu::Stm::Timers::reset<AdcTimerNumber>();
                    adcTimer->PSC = prescaler;
                    adcTimer->ARR = period / 100;
                    // adcTimer->ARR = period / 200;

                    adcTimer->SMCR |= (0b001 << TIM_SMCR_SMS_Pos); // gated mode + reset
                    adcTimer->SMCR |= TIM_SMCR_SMS_3;

                    // pwmTimer: cc1 / cc2 as trgo
                    MODIFY_REG(pwmTimer->CR2, TIM_CR2_MMS_Msk, (0b0100 << TIM_CR2_MMS_Pos));

                    adcTimer->SMCR |= (Timers::trgoToTrigger<PwmTimerNumber, AdcTimerNumber>() << TIM_SMCR_TS_Pos); //TIM3!!!!
                    adcTimer->CR1 |= TIM_CR1_ARPE;
                    adcTimer->CR2 |= (0b0010 << TIM_CR2_MMS_Pos); // update as trgo

                    // adcTimer->DIER |= TIM_DIER_UIE;

                    adcTimer->CR1 |= TIM_CR1_CEN;
                }
                static inline void setSingleMode() {
                    Mcu::Stm::Timers::reset<AdcTimerNumber>();
                    adcTimer->PSC = prescaler;
                    adcTimer->ARR = period;

                    adcTimer->SMCR |= (0b000 << TIM_SMCR_SMS_Pos);
                    adcTimer->SMCR |= TIM_SMCR_SMS_3;

                    MODIFY_REG(pwmTimer->CR2, TIM_CR2_MMS_Msk, (0b0010 << TIM_CR2_MMS_Pos));// update as trigger-output

                    adcTimer->SMCR |= (Timers::trgoToTrigger<PwmTimerNumber, AdcTimerNumber>() << TIM_SMCR_TS_Pos); //TIM3!!!!

                    adcTimer->CCMR1 |= (0b1000 << TIM_CCMR1_OC1M_Pos); // retriggerable OPM
                    adcTimer->CR1 |= TIM_CR1_ARPE;
                    // adcTimer->CR2 |= (0b0100 << TIM_CR2_MMS_Pos); // cc1 as trgo
                    adcTimer->CR2 |= (0b0011 << TIM_CR2_MMS_Pos); // CC1IF trgo

                    // adcTimer->DIER |= TIM_DIER_CC1IE; // test only

                    adcTimer->CR1 |= TIM_CR1_CEN;
                }
                static inline uint16_t pwmFreq() {
                    return freq;
                }
                static inline uint16_t maxDuty() {
                    return period;
                }
                static inline void pwm(const uint16_t f) {
                    freq = f;
                    prescaler = (Clock::config::frequency.value / (freq * period));
                    pwmTimer->PSC = prescaler;
                    pwmTimer->EGR |= TIM_EGR_UG;
                    adcTimer->PSC = prescaler;
                    adcTimer->EGR |= TIM_EGR_UG;
                }
                static inline void duty(const uint16_t v) {
                    if (v <= period) {
                        pwmTimer->CCR1 = v;
                        pwmTimer->CCR2 = v;
                        adcTimer->CCR1 = std::max(1.0f, (triggerTiming * v)); // trigger
                    }
                }
                template<template<auto, auto> typename T, auto L, auto U>
                static inline void duty(const T<L, U>& v) {
                    if (v) {
                        const uint16_t vv = period * ((float)v.toInt() - L) / (U - L);
                        pwmTimer->CCR1 = vv;
                        pwmTimer->CCR2 = vv;
                        adcTimer->CCR1 = std::max(1.0f, (triggerTiming * vv)); // trigger
                    }
                }


                static inline void trigger(const float v) {
                }

                constexpr static inline uint8_t trgo() {
                    if constexpr(AdcTimerNumber == 3) {
                        return 4; // tim3-trgo
                    }
                    else if constexpr(AdcTimerNumber == 2) {
                        return 11; // tim2-trgo
                    }
                    else if constexpr(AdcTimerNumber == 4) {
                        return 12; // tim4-trgo
                    }
                }
                private:
                static inline uint32_t period = 1640;
                static inline uint32_t freq   = 24000;
                static inline uint16_t prescaler = (Clock::config::frequency.value / (freq * period)) - 1;
                static inline float triggerTiming = 0.9f;
            };
        }

        template<uint8_t TimerNumber, typename Clock, typename MCU = DefaultMcu>
        struct Bdc2 {
            static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Mcu::Components::Timer<TimerNumber>>::value);

            static inline uint32_t period = 1640;
            static inline uint32_t freq   = 24000;
            static inline uint16_t prescaler = (Clock::config::frequency.value / (freq * period)) - 1;

            // std::integral_constant<uint16_t, prescaler>::_;

            static inline void init() {
                if constexpr (TimerNumber == 1) {
                    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
                }
                else if constexpr (TimerNumber == 2) {
                    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
                }
                else if constexpr (TimerNumber == 3) {
                    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
                }
                else if constexpr (TimerNumber == 4) {
                    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
                }
                else {
                    static_assert(false);
                }
                mcuTimer->PSC = prescaler;
                mcuTimer->ARR = period;
                mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC1M_Pos); // pwm1
                // mcuTimer->CCMR1 |= TIM_CCMR1_OC1PE;
                mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC2M_Pos); // pwm2
                // mcuTimer->CCMR1 |= TIM_CCMR1_OC2PE;
                mcuTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC3M_Pos); // pwm3
                // mcuTimer->CCMR2 |= TIM_CCMR2_OC3PE;
                mcuTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC4M_Pos); // pwm4
                // mcuTimer->CCMR2 |= TIM_CCMR2_OC4PE;
                mcuTimer->CCER |= TIM_CCER_CC1E;
                mcuTimer->CCER |= TIM_CCER_CC2E;

                mcuTimer->CCER |= TIM_CCER_CC2P;

                mcuTimer->CCR1 = 0;
                mcuTimer->CCR2 = 0;
                mcuTimer->CCR3 = 1;
                mcuTimer->CR1 |= TIM_CR1_ARPE;
                mcuTimer->EGR |= TIM_EGR_UG;

                mcuTimer->CR2 |= (0b110 << TIM_CR2_MMS_Pos); // ch3 as trigger-output

                mcuTimer->CR1 |= TIM_CR1_CEN;
            }

            static inline void pwm(const uint16_t f) {
                freq = f;
                prescaler = (Clock::config::frequency.value / (freq * period)) - 1;
                mcuTimer->PSC = prescaler;
                mcuTimer->EGR |= TIM_EGR_UG;
            }

            static inline void duty(const uint16_t v) {
                mcuTimer->CCR1 = v;
                mcuTimer->CCR2 = v;
                mcuTimer->CCR3 = std::max(1.0f, (9.0f * v) / 10.0f); // trigger
            }

            constexpr static inline uint8_t trgo() {
                return 4; // tim3-trgo
            }
        };


        template<uint8_t TimerNumber, typename Clock, typename MCU = DefaultMcu>
        struct Bdc {
            static inline /*constexpr */ TIM_TypeDef* const mcuTimer = reinterpret_cast<TIM_TypeDef*>(Mcu::Stm::Address<Timer<TimerNumber, void, void, MCU>>::value);

            static inline constexpr uint16_t period = 1640;
            static inline uint32_t freq  = 24000;
            static inline uint16_t prescaler = (Clock::config::frequency.value / (freq * period));
            static inline float triggerTiming = 0.9f;

            static inline void init() {
                if constexpr (TimerNumber == 1) {
                    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
                }
                else if constexpr (TimerNumber == 2) {
                    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
                }
                else if constexpr (TimerNumber == 3) {
                    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
                }
                else if constexpr (TimerNumber == 4) {
                    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
                }
                else {
                    static_assert(false);
                }
                mcuTimer->PSC = prescaler;
                mcuTimer->ARR = period;
                mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC1M_Pos); // pwm1
                // mcuTimer->CCMR1 |= TIM_CCMR1_OC1PE;
                mcuTimer->CCMR1 |= (0b0110 << TIM_CCMR1_OC2M_Pos); // pwm2
                // mcuTimer->CCMR1 |= TIM_CCMR1_OC2PE;
                mcuTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC3M_Pos); // pwm3
                // mcuTimer->CCMR2 |= TIM_CCMR2_OC3PE;
                mcuTimer->CCMR2 |= (0b0110 << TIM_CCMR2_OC4M_Pos); // pwm4
                // mcuTimer->CCMR2 |= TIM_CCMR2_OC4PE;
                mcuTimer->CCER |= TIM_CCER_CC1E;
                mcuTimer->CCER |= TIM_CCER_CC2E;

                mcuTimer->DIER |= TIM_DIER_CC3IE; // TEST

                mcuTimer->CCR1 = 0;
                mcuTimer->CCR2 = 0;
                mcuTimer->CCR3 = 1;
                mcuTimer->CR1 |= TIM_CR1_ARPE;

                mcuTimer->CR2 |= (0b110 << TIM_CR2_MMS_Pos); // ch3 as trigger-output

                mcuTimer->CR1 |= TIM_CR1_CEN;
            }

            static inline void pwm(const uint16_t f) {
                freq = f;
                prescaler = (Clock::config::frequency.value / (freq * period));
                mcuTimer->PSC = prescaler;
                mcuTimer->EGR |= TIM_EGR_UG;
            }

            static inline void duty(const uint16_t v) {
                mcuTimer->CCR1 = v;
                mcuTimer->CCR2 = v;
                mcuTimer->CCR3 = std::max(1.0f, (triggerTiming * v)); // trigger
            }
            static inline void trigger(const float v) {
                triggerTiming = v;
            }

            constexpr static inline uint8_t trgo() {
                if constexpr(TimerNumber == 3) {
                    return 4; // tim3-trgo
                }
                else if constexpr(TimerNumber == 2) {
                    return 11; // tim2-trgo
                }
                else if constexpr(TimerNumber == 4) {
                    return 12; // tim4-trgo
                }
            }
        };

    }
}
