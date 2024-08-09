#pragma once

#include <cstring>

#include "meta.h"

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "mcu/eeprom.h"

#include "pwm.h"
#include "usart.h"
#include "si5351.h"
#include "i2c.h"
#include "timer.h"
#include "opamp.h"
#include "adc.h"
#include "dac.h"
#include "clock.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "gpio.h"
#include "tick.h"
#include "meta.h"
#include "rc/rc.h"
#include "rc/crsf.h"
#include "rc/sport.h"
#include "rc/roboremo.h"
#include "rc/escape.h"
#include "rc/sbus2.h"
#include "rc/vesc.h"
#include "rc/gps.h"

struct Pult00;

using namespace Mcu::Stm;
using namespace std::literals::chrono_literals;

template<typename HW, typename MCU = DefaultMcu>
struct Devices;

template<>
struct Devices<Pult00, Mcu::Stm::Stm32G473> {
    using MCU = Mcu::Stm::Stm32G473;

    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 2_MHz, 1024>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;


    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        gpioa::init();
        gpiob::init();
        gpioc::init();
        gpiof::init();

    }
};

