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

#include "mcu/avr8.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/pinchange.h"
#include "mcu/ports.h"
#include "mcu/avr/delay.h"
#include "mcu/i2cslave.h"
#include "hal/alarmtimer.h"
#include "hal/softtimer.h"
#include "hal/event.h"
#include "hal/alarmtimer.h"
#include "hal/constantrate.h"
#include "external/ws2812.h"
#include "external/rpm.h"

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using ledPin = AVR::Pin<PortB, 4>;
using led = WS2812<2, ledPin>;
typedef led::color_type Color;

using reflexPin = AVR::Pin<PortB, 3>;
using reflexPinSet = AVR::PinSet<reflexPin>;
using reflexPinChange = AVR::PinChange<reflexPinSet>;

using hwTimer = AVR::Timer8Bit<0>; 
using rpmTimer = SoftTimer<hwTimer, uint16_t>;

constexpr std::RPM MaximumRpm{12000};
constexpr std::RPM MinimumRpm{100};
using rpm = RpmFromInterruptSource<reflexPinChange, rpmTimer, MaximumRpm, MinimumRpm>;

using systemTimer = AVR::Timer8Bit<1>;
using alarmTimer = AlarmTimer<systemTimer>;

using systemConstantRate = ConstantRateAdapter<1, systemTimer, AVR::ISR::Timer<1>::CompareA, alarmTimer>;

static auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);

constexpr TWI::Address address{0x55};
using Usi = AVR::Usi<0>;
using i2c = I2C::I2CSlave<Usi, address, 2>;

using isrRegistrar = IsrRegistrar<systemConstantRate, i2c::I2CSlaveHandlerOvfl, i2c::I2CSlaveHandlerStart, rpmTimer, rpm>;

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(std::byte b) {
        auto t = std::to_integer<uint7_t>(b);
        if (t == *periodicTimer) {
            static uint8_t counter = 0;
            if (++counter == 0) {
                led::set(Color{Green{16}});
            }
            else {
                led::set(Color{Blue{16}});
            }
            return true;
        }
        return false;
    }
};

int main() 
{
    isrRegistrar::init();
    i2c::init();
    
    alarmTimer::init();
//    systemConstantRate::init();
    
    led::init();
    led::off();

//    rpm::init();
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        using eventHandler = EventHandlerGroup<TimerHandler>;
        
        EventManager::run2<eventHandler>([]() {
            alarmTimer::periodic();
            auto r = rpm::rpm();
            if (r) {
                i2c::registers()[0] = r.value();
                i2c::registers()[1] = r.value() >> 8;
            }
        });
    }
}
ISR(PCINT0_vect) {
    isrRegistrar::isr<AVR::ISR::PcInt<0>>();
}
ISR(TIMER0_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::Overflow>();
}
//ISR(TIMER1_OVF_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<1>::Overflow>();
//}
ISR(TIMER1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}
ISR(USI_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Usi<0>::Overflow>();
}
ISR(USI_START_vect) {
    isrRegistrar::isr<AVR::ISR::Usi<0>::Start>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
    while(true) {}
}
#endif