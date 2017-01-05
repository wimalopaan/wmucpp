/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "console.h"
#include "mcu/avr/isr.h"
#include "mcu/ports.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/util.h"
#include "hal/alarmtimer.h"
#include "util/disable.h"

#include "simavr/simavrdebugconsole.h"

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using led = AVR::Pin<PortB, 0>;

using systemClock = AVR::Timer8Bit<0>;


using isrReg = IsrRegistrar<>;

int main() {
    Set<led>::output();
    constexpr auto timerParameter = AVR::Util::calculate<systemClock>(80_Hz);
    static_assert(timerParameter, "wrong timer parameter");
    
    // todo: systemClock::setup<timerParameter>()
    
    systemClock::prescale<timerParameter.prescaler>();
    systemClock::ocra<timerParameter.ocr>();
    
    {
        Scoped<EnableInterrupt> ie;
        while(true) {
            
        }
    }
}

ISR(TIMER0_COMPA_vect) {
//    isrReg::isr<AVR::ISR::Timer<0>::CompareA>();
}
