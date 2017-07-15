/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "mcu/avr8.h"
#include "mcu/avr/util.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/mcupwm.h"
#include "mcu/ports.h"
#include "mcu/avr/isr.h"
#include "hal/event.h"
#include "units/percent.h"

struct TLE5205Base {
    struct Direction {
        bool isClockWise = true;
    };
};

template<uint8_t TimerN, typename ErrPin, typename MCU = DefaultMcuType>
class TLE5205Hard : public TLE5205Base {
public:
    using mcu_pwm = AVR::PWM<TimerN, MCU>;
    typedef typename mcu_pwm::value_type value_type;
    
    typedef ErrPin err_pin;

    template<const std::hertz& MinFrequency>
    static void init() {
        mcu_pwm::template init<MinFrequency>();        
        
        if constexpr(!std::is_same<err_pin, void>::value) {
            err_pin::template dir<AVR::Input>();
            err_pin::pullup();
        }
    }
    static void pwm(const std::percent& p) {
        if (direction().isClockWise) {
            mcu_pwm::template pwm<typename mcu_pwm::A>(p);
            mcu_pwm::template pwm<typename mcu_pwm::B>(std::percent{0});
        }
        else {
            mcu_pwm::template pwm<typename mcu_pwm::A>(p);
            mcu_pwm::template pwm<typename mcu_pwm::B>(p);
        }
    }
    static value_type timerValue() {
        return mcu_pwm::template ocr<typename mcu_pwm::B>();
    }
    static std::hertz frequency() {
        return mcu_pwm::frequency();
    }
    static Direction& direction() {
        static Direction dir;
        return dir;
    }
private:
};

template<typename InPin1, typename InPin2, typename ErrPin, typename Timer>
class TLE5205Soft : public TLE5205Base {
public:
    
    typedef InPin1 pin1_pin;
    typedef InPin2 pin2_pin;
    typedef ErrPin err_pin;
    typedef Timer mcu_timer_type;
    typedef typename Timer::value_type value_type;
    typedef typename Timer::flags_type flags_type;
    typedef typename Timer::mask_type mask_type;
    
    using pinset = AVR::PinSet<InPin1, InPin2>;
    
    struct PwmOnHandler : public IsrBaseHandler<typename AVR::ISR::Timer<Timer::number>::Overflow> {
        static void isr() {
            if (direction().isClockWise) {
                pinset::template off<InPin1, InPin2>();
            }
            else {
                pinset::template off<InPin1>();
            }
        }
    };
    struct PwmOffHandler : public IsrBaseHandler<typename AVR::ISR::Timer<Timer::number>::CompareA> {
        static void isr() {
            pinset::template on<InPin1, InPin2>();
        }
    };
    
    static Direction& direction() {
        static Direction dir;
        return dir;
    }
    static void pwm(const std::percent& p) {
        using namespace std::literals::quantity;
        if (p > 0_ppc) {
            Timer::mcuInterrupts()->timsk.template add<mask_type::ociea | mask_type::toie>();
            Timer::ocra(std::expand(p, value_type{0}, std::numeric_limits<value_type>::max()));        
        }
        else {
            Timer::mcuInterrupts()->timsk.template clear<mask_type::ociea | mask_type::toie>();
            pinset::template on<InPin1, InPin2>();
        }
    }
    
    template<const std::hertz& MinFrequency>
    static void init() {
        constexpr auto prescaler = AVR::Util::prescalerForAbove<Timer>(MinFrequency);
        static_assert(prescaler > 0, "wrong prescaler");
        Timer::template prescale<prescaler>();
        
        Timer::mode(AVR::TimerMode::Normal);        
        Timer::ocra(0);        
        pinset::template dir<AVR::Output>();
        pinset::allOn();
        if constexpr(!std::is_same<err_pin, void>::value) {
            err_pin::template dir<AVR::Input>();
            err_pin::pullup();
        }
    }
    
    static void periodic() {
        if (!err_pin::read()) {
            // todo: Status: direction , error in ein byte
//            EventManager::enqueue({EventType::TLE5205Error, 0}); 
        }
    }
private:
};

