#pragma once

#include "clock.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "mcu_traits.h"
#include "arm.h"
#include "gpio.h"

template<typename HW, typename MCU = void>
struct Devices {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 1000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 
//    using systemTimer = STM32::SystemTimer<clock, UseInterrupts<true>, MCU>; 
    using trace = Arm::Trace<clock>;
    
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B>;
    using pinb8 = Mcu::Stm::Pin<gpiob, 8>;
    using pinb4 = Mcu::Stm::Pin<gpiob, 4>;
    
    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();
        
        gpiob::init();
        pinb8::dir<Mcu::Output>();
        pinb4::dir<Mcu::Output>();
        pinb4::speed<Mcu::High>();
    }
};

