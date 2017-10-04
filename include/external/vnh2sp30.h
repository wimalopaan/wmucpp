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

template<typename PWM, typename Channel>
struct PwmAdapter {
    static void pwm(const std::percent& value) {
        PWM::template pwm<Channel>(value);
    }
};

template<typename PWM, typename DirPin>
class VNH {
public:
    
    struct CW {};
    struct CCW {};
    
    static void init() {
        using namespace std::literals::quantity;
        direction<CW>();
        pwm(0_ppc);
    }
    
    static void pwm(const std::percent& value) {
        PWM::pwm(value);
    }
    template<typename Dir>
    static void direction() {
        if constexpr(std::is_same<Dir, CW>::value) {
            DirPin::on();
        }
        else {
            DirPin::off();
        }
    }

private:
};
