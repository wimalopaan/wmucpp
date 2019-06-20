/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2019 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <etl/scoped.h>
#include <etl/meta.h>

namespace External::Solutions {
    
    template<typename lowA, typename highA, typename lowB, typename highB, typename PWM, const auto& fPwm1, const auto& fPwm2>
    struct Bridge final {
        
        using pwm = PWM;
        using pwm_type = typename PWM::value_type;
        
        inline static constexpr pwm_type beepLevel = 10;
        
        inline static void init() {
            lowA::init();
            highA::init();
            lowB::init();
            highB::init();
        }    
        
        inline static void lowSideOff() {
            lowA::inactivate();
            lowB::inactivate();
        }
        inline static void highSideOff() {
            highA::inactivate();
            highB::inactivate();
        }
        
        inline static void allOff() {
            lowSideOff();
            highSideOff();
        }
        struct OCAHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Timer<PWM::number>::CompareA> {
            inline static void isr() {
                lowSideOff();
            }
        };
        struct OCBHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Timer<PWM::number>::CompareB> {
            inline static void isr() {
                switch(beepPhase) {
                case 0:
                    lowA::inactivate();
                    highB::activate();
                    break;
                case 1:
                    lowA::inactivate();
                    highB::inactivate();
                    backward = true;
                    break;
                case 2:
                    lowB::inactivate();
                    highA::activate();
                    break;
                case 3:
                    lowB::inactivate();
                    highA::inactivate();
                    backward = false;
                    break;
                }
                ++beepPhase;
            }
        };
        struct OVFHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Timer<PWM::number>::Overflow>  {
            inline static void isr() {
                if (backward) {
                    lowB::activate();
                }
                else {
                    lowA::activate();
                }
            }
        };
        
        static inline void beep(bool on) {
            etl::Scoped<etl::DisbaleInterrupt<etl::ForceOn>> di;
            if (on) {
                pwm::off();
                allOff();
                pwm::template f<fPwm2>();
                pwm::b(20);
                pwm::enable(AVR::PWM::Overflow<true>());
                pwm::enable(AVR::PWM::ChannelB<true>());
                backward = false;
                beepPhase = 0;
            }
            else {
                pwm::off();
                allOff();
                pwm::template f<fPwm1>();
                backward = false;
            }
        }
        
        static inline void off() {
            etl::Scoped<etl::DisbaleInterrupt<etl::ForceOn>> di;
            pwm::off();
            allOff();
        }
        static inline void on() {
            etl::Scoped<etl::DisbaleInterrupt<etl::ForceOn>> di;
            pwm::enable(AVR::PWM::ChannelA<true>());
            pwm::enable(AVR::PWM::Overflow<true>());
        }
        
        template<typename InputType>
        static inline void set(const InputType& d, bool back) {
            using value_type = typename InputType::value_type;
            constexpr value_type span = (InputType::Upper - InputType::Lower);
            //        InputType::_;
            //        std::integral_constant<uint16_t, pwm::template max<fPWM>()>::_;
            
            if (!d) return;
            
            value_type t = (etl::enclosing_t<value_type>(d.toInt()) * std::numeric_limits<pwm_type>::max()) / span;
            
            value_type lastt = 0;
            
            if (t == lastt) return;
            
            lastt = t;
            
            if (t < 20) {
                off();
            }
            else {
                on();
            }
            if (t < (std::numeric_limits<pwm_type>::max() - 20)){
                etl::Scoped<etl::DisbaleInterrupt<etl::ForceOn>> di;
                backward = back;        
                pwm::a(t);
                
                if (backward) {
                    highB::inactivate();
                    lowA::inactivate();
                    highA::activate();
                }
                else {
                    highA::inactivate();
                    lowB::inactivate();
                    highB::activate();
                }
                mFull = false;
            }
            else {
                mFull = true;
                etl::Scoped<etl::DisbaleInterrupt<etl::ForceOn>> di;
                backward = back;        
                off();
                if (backward) {
                    highB::inactivate();
                    lowA::inactivate();
                    lowB::activate();
                    highA::activate();
                }
                else {
                    lowB::inactivate();
                    highA::inactivate();
                    highB::activate();
                    lowA::activate();
                }
            }
        }
    private:
        static inline volatile bool mFull = false;
        static inline volatile bool backward = false;
        static inline volatile etl::uint_ranged_circular<uint8_t, 0, 3> beepPhase;
    };
    
    template<typename Bridge, auto PulseCount = 10u>
    struct MotorBeeper {
        using dev = Bridge;
        static inline void periodic() {
            if (mOn) {
                ++mCount;
                if (mCount == mPulseCount) {
                    dev::beep(false);
                }
                if (mCount >= (2 * mPulseCount)) {
                    dev::beep(true);
                    mCount = 0;
                }
            }
        }
        static inline void beep(bool on) {
            mOn = on;
            mCount = 0;
            dev::beep(on);
        }
        static inline void pulse(uint16_t c) {
            mPulseCount = c;
        }
    private:
        static inline bool mOn = false;
        static inline uint16_t mCount = 0;
        static inline uint16_t mPulseCount = PulseCount;
    };
    
}
