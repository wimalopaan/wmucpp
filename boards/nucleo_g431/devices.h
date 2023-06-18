#pragma once

#include "mcu.h"
#include "adc.h"
#include "dac.h"
#include "clock.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "mcu_traits.h"
#include "arm.h"
#include "gpio.h"
#include "tick.h"

template<typename HW, typename MCU = void>
struct Devices {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 1000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 
//    using systemTimer = STM32::SystemTimer<clock, UseInterrupts<true>, MCU>; 
    using trace = Arm::Trace<clock>;
    
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using pinb8 = Mcu::Stm::Pin<gpiob, 8, MCU>;
    using pinb4 = Mcu::Stm::Pin<gpiob, 4, MCU>;
    
    using dac1 = Mcu::Stm::Dac<1, MCU>;
    using adc1 = Mcu::Stm::Adc<1, MCU>;
    
    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();
        
        gpiob::init();
        pinb8::template dir<Mcu::Output>();
        pinb4::template dir<Mcu::Output>();
        pinb4::template speed<Mcu::High>();
        
        dac1::init();
        adc1::init();
    }
};

