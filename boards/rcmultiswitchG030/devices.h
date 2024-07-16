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

struct Nucleo;
struct SW01;

template<typename HW, typename Config, typename MCU = DefaultMcu>
struct Devices;

template<typename Config, typename MCU>
struct Devices<SW01, Config, MCU> {
    using mcuconfig = Config::mcuconfig;

    using clock = mcuconfig::clock;

    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    // using data = Config::data;
    using crsfcallback = Config::crsfcallback;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpioa, 2, MCU>;
    // CRSF RX
    using crsfrx = Mcu::Stm::Pin<gpioa, 3, MCU>;

    // Usart 2: CRSF
    struct CrsfAdapterConfig;
    using crsf_pa = RC::Protokoll::Crsf::Adapter<0, CrsfAdapterConfig, MCU>;
    using crsf    = Mcu::Stm::Uart<2, crsf_pa, RC::Protokoll::Crsf::maxMessageSize, std::byte, clock, MCU>;
    using crsf_out= RC::Protokoll::Crsf::Generator<crsf, systemTimer, MCU>;

    struct CrsfAdapterConfig {
        using out = crsf_out;
        using callback = Config::crsfcallback;
        using debug = crsf;
    };

    // PA8 Led
    using led = Mcu::Stm::Pin<gpioa, 8, MCU>;
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

    static inline void init() {
        clock::init();
        systemTimer::init();

        // SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        gpioa::init();
        gpiob::init();
        gpioc::init();

        led::template dir<Mcu::Output>();

        crsf::init();
        crsf::baud(420'000);
        crsftx::afunction(7);
        crsftx::template pullup<true>();
        crsfrx::afunction(7);

    }
};

template<typename Config, typename MCU>
struct Devices<Nucleo, Config, MCU> {
    using mcuconfig = Config::mcuconfig;

    using clock = mcuconfig::clock;

    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    // using data = Config::data;
    using crsfcallback = Config::crsfcallback;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpiob, 6, MCU>;
    // CRSF RX
    using crsfrx = Mcu::Stm::Pin<gpiob, 7, MCU>;

    // Usart 2: CRSF
    struct CrsfAdapterConfig;
    using crsf_pa = RC::Protokoll::Crsf::Adapter<0, CrsfAdapterConfig, MCU>;
    using crsf    = Mcu::Stm::Uart<1, crsf_pa, RC::Protokoll::Crsf::maxMessageSize, char, clock, MCU>;
    using crsf_out= RC::Protokoll::Crsf::Generator<crsf, systemTimer, MCU>;


    using debugtx = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using debugrx = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using debug   = Mcu::Stm::Uart<2, void, 256, char, clock, MCU>;

    struct CrsfAdapterConfig {
        using out = crsf_out;
        using callback = Config::crsfcallback;
        using debug = debug;
    };

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

    using pb4 = Mcu::Stm::Pin<gpiob, 4, MCU>;

    static inline void init() {
        clock::init();
        systemTimer::init();

        gpioa::init();
        gpiob::init();
        gpioc::init();

        led::template dir<Mcu::Output>();
        pb4::template dir<Mcu::Output>();

        crsf::init();
        crsf::baud(420'000);
        crsftx::afunction(0);
        crsftx::template pullup<true>();
        crsfrx::afunction(0);

        debug::init();
        debug::baud(420'000);
        debugtx::afunction(1);
        debugtx::template pullup<true>();
        debugrx::afunction(1);

    }
};

