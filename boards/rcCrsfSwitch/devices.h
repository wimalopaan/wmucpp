#pragma once

#include <cstring>

#include "etl/fixedvector.h"
#include "etl/stackstring.h"

#include "meta.h"

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "components.h"
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

template<typename HW, typename Config, typename MCU = DefaultMcu>
struct Devices;

template<typename Config, typename MCU>
struct Devices<SW01, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpioa, 9, MCU>; // AF 1
    // CRSF RX
    using crsfrx = Mcu::Stm::Pin<gpioa, 10, MCU>; // AF1
    // Usart 1: CRSF
    struct CrsfAdapterConfig;
    using crsf_pa = RC::Protokoll::Crsf::Adapter<0, CrsfAdapterConfig, MCU>;
    using crsf    = Mcu::Stm::Uart<1, crsf_pa, RC::Protokoll::Crsf::maxMessageSize, std::byte, clock, MCU>;
    using crsf_out= RC::Protokoll::Crsf::Generator<crsf, systemTimer, MCU>;

    // LPUART 1: CRSF HD1
    using crsf_hd1_rxtx = Mcu::Stm::Pin<gpioa, 2, MCU>; // AF 6

    // USART 2: CRSF HD2
    using crsf_hd2_rxtx = Mcu::Stm::Pin<gpioa, 14, MCU>; // AF 1

    // USART 3: CRSF HD3
    using crsf_hd3_rxtx = Mcu::Stm::Pin<gpioa, 5, MCU>; // AF 4

    // USART 4: CRSF HD4
    using crsf_hd4_rxtx = Mcu::Stm::Pin<gpioa, 0, MCU>; // AF 4

    // USART 5: CRSF HD5
    using crsf_hd5_rxtx = Mcu::Stm::Pin<gpiob, 0, MCU>; // AF 8

    // USART 6: CRSF HD6
    using crsf_hd6_rxtx = Mcu::Stm::Pin<gpioa, 4, MCU>; // AF 3

    // LPUART 2: CRSF HD7
    using crsf_hd7_rxtx = Mcu::Stm::Pin<gpiob, 6, MCU>; // AF 10

    // Led
    using led = Mcu::Stm::Pin<gpiob, 9, MCU>;

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
        led::set();

        crsf::init();
        crsf::baud(420'000);
        crsf::halfDuplex();
        crsftx::afunction(1);
        crsftx::template pullup<true>();
        crsfrx::afunction(1);

    }
};


