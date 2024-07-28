#pragma once

#include <cstring>

#include "etl/fixedvector.h"
#include "etl/stackstring.h"

#include "meta.h"

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "components.h"
#include "pwm.h"
#include "usart.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "clock.h"
#include "gpio.h"
#include "tick.h"
#include "meta.h"
#include "rc/rc.h"
#include "rc/crsf.h"

struct SW01;

template<typename HW, template<typename, typename> typename CrsfCallback, typename MCU = DefaultMcu>
struct Devices;

template<template<typename, typename> typename CrsfCallback, typename MCU>
struct Devices<SW01, CrsfCallback, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using mco = Mcu::Stm::Pin<gpioa, 8, MCU>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpiob, 6, MCU>;
    // CRSF RX
    using crsfrx = Mcu::Stm::Pin<gpiob, 7, MCU>;

    // Usart 2: CRSF
    struct CrsfAdapterConfig;
    using crsf_pa = RC::Protokoll::Crsf::Adapter<0, CrsfAdapterConfig, MCU>;
    using crsf    = Mcu::Stm::Uart<1, crsf_pa, RC::Protokoll::Crsf::maxMessageSize, std::byte, clock, MCU>;
    using crsf_out= RC::Protokoll::Crsf::Generator<crsf, systemTimer, MCU>;

    // Led
    using led = Mcu::Stm::Pin<gpioc, 6, MCU>;
    struct LedCallback {
        static inline void on(const bool on) {
            if (on) {
                led::set();
            }
            else {
                led::reset();
            }
        }
    };

    struct CrsfAdapterConfig {
        using out = crsf_out;
        // using callback = CrsfCallback<bsws, void>;
        // using debug = debug;
    };


    static inline void init() {
        clock::init();
        systemTimer::init();

        gpioa::init();
        gpiob::init();
        gpioc::init();

        led::template dir<Mcu::Output>();

        crsf::init();
        crsf::baud(420'000);
        crsf::halfDuplex();
        crsftx::afunction(0);
        crsftx::template pullup<true>();
        crsfrx::afunction(0);

    }
};


