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

#include "blinker.h"
#include "adapter.h"

struct Nucleo;
struct SW01;

template<typename HW, template<typename, typename> typename CrsfCallback, typename MCU = DefaultMcu>
struct Devices2;

template<template<typename, typename> typename CrsfCallback, typename MCU>
struct Devices2<Nucleo, CrsfCallback, MCU> {
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

    using debugtx = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using debugrx = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using debug   = Mcu::Stm::Uart<2, void, 256, char, clock, MCU>;
    // using debug   = Config::debug;

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

    using sw0 = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using sw1 = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using sw2 = Mcu::Stm::Pin<gpioa, 4, MCU>; // tim 14 ch1
    using sw3 = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using sw4 = Mcu::Stm::Pin<gpioa, 12, MCU>;
    using sw5 = Mcu::Stm::Pin<gpioa, 11, MCU>; // tim 1 ch4
    using sw6 = Mcu::Stm::Pin<gpioa, 6, MCU>; // tim3 ch1
    using sw7 = Mcu::Stm::Pin<gpioa, 7, MCU>; // tim3 ch2

    using pwm3 = Mcu::Stm::V2::Pwm::Simple<3, clock>;

    using debug1 = void;
    using adap30 = Local::PwmAdapter<pwm3, 0, debug1>;
    using adap31 = Local::PwmAdapter<pwm3, 1, debug1>;

    using bsw0 = Local::BlinkerWithPwm<sw0, systemTimer, void, debug1>;
    using bsw1 = Local::BlinkerWithPwm<sw1, systemTimer, void, debug1>;
    using bsw2 = Local::BlinkerWithPwm<sw2, systemTimer, void, debug1>;
    using bsw3 = Local::BlinkerWithPwm<sw3, systemTimer, void, debug1>;
    using bsw4 = Local::BlinkerWithPwm<sw4, systemTimer, void, debug1>;
    using bsw5 = Local::BlinkerWithPwm<sw5, systemTimer, void, debug1>;
    using bsw6 = Local::BlinkerWithPwm<sw6, systemTimer, adap30, debug1>;
    using bsw7 = Local::BlinkerWithPwm<sw7, systemTimer, adap31, debug1>;

    using bsws = Meta::List<bsw0, bsw1, bsw2, bsw3, bsw4, bsw5, bsw6, bsw7>;

    struct CrsfAdapterConfig {
        using out = crsf_out;
        using callback = CrsfCallback<bsws, void>;
        using debug = debug;
    };


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
        crsf::halfDuplex();
        crsftx::afunction(0);
        crsftx::template pullup<true>();
        crsfrx::afunction(0);

        debug::init();
        debug::baud(420'000);
        debugtx::afunction(1);
        debugtx::template pullup<true>();
        debugtx::openDrain();
        debugrx::afunction(1);

        // mco::template dir<Mcu::Output>();
        // mco::speed();
        // mco::afunction(0);

        sw0::template dir<Mcu::Output>();
        sw1::template dir<Mcu::Output>();
        sw2::template dir<Mcu::Output>();
        sw3::template dir<Mcu::Output>();
        sw4::template dir<Mcu::Output>();
        sw5::template dir<Mcu::Output>();
        sw6::template dir<Mcu::Output>();
        sw7::template dir<Mcu::Output>();

        pwm3::init();
    }
};


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
    // using crsfcallback = Config::crsfcallback;

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


    using debugtx = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using debugrx = Mcu::Stm::Pin<gpioa, 3, MCU>;
    // using debug   = Mcu::Stm::Uart<2, void, 256, char, clock, MCU>;
    using debug   = Config::debug;

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

    using sw0 = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using sw1 = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using sw2 = Mcu::Stm::Pin<gpioa, 4, MCU>; // tim 14 ch1
    using sw3 = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using sw4 = Mcu::Stm::Pin<gpioa, 12, MCU>;
    using sw5 = Mcu::Stm::Pin<gpioa, 11, MCU>; // tim 1 ch4
    using sw6 = Mcu::Stm::Pin<gpioa, 6, MCU>; // tim3 ch1
    using sw7 = Mcu::Stm::Pin<gpioa, 7, MCU>; // tim3 ch2

    using pwm3 = Mcu::Stm::V2::Pwm::Simple<3, clock>;

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
        crsf::halfDuplex();
        crsftx::afunction(0);
        crsftx::template pullup<true>();
        crsfrx::afunction(0);

        debug::init();
        debug::baud(420'000);
        debugtx::afunction(1);
        debugtx::template pullup<true>();
        debugtx::openDrain();
        debugrx::afunction(1);

        // mco::template dir<Mcu::Output>();
        // mco::speed();
        // mco::afunction(0);

        sw0::template dir<Mcu::Output>();
        sw1::template dir<Mcu::Output>();
        sw2::template dir<Mcu::Output>();
        sw3::template dir<Mcu::Output>();
        sw4::template dir<Mcu::Output>();
        sw5::template dir<Mcu::Output>();
        sw6::template dir<Mcu::Output>();
        sw7::template dir<Mcu::Output>();

        pwm3::init();
    }
};

