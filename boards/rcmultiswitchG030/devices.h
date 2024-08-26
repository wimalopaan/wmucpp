/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 -2024 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <cstring>

#include "etl/fixedvector.h"
#include "etl/stackstring.h"

#include "meta.h"
#include "etl/algorithm.h"
#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "mcu/eeprom.h"
#include "components.h"
#include "pwm.h"
#include "pwm_dma.h"
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
#include "button.h"

struct Nucleo;
struct SW01;
struct SW10; // board: RCMultiSwitchSmall10
struct SW11; // board: RCMultiSwitchSmall10 // wrong dma (does not work with GPIO on G0xx!)
struct SW12; // board: RCMultiSwitchSmall10
struct SW13; // board: RCMultiSwitchSmall10

struct SW99; // test

using namespace std::literals::chrono_literals;

template<typename HW, template<typename, typename> typename CrsfCallback, typename MCU = DefaultMcu>
struct Devices2;

template<template<typename, typename> typename CrsfCallback, typename MCU>
struct Devices2<SW99, CrsfCallback, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    // using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpioa, 2, MCU>;

    // Usart 2: CRSF

    using crsf    = Mcu::Stm::Uart<2, void, 2, std::byte, clock, MCU>;

    using debugtx = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using debugrx = Mcu::Stm::Pin<gpiob, 7, MCU>;
    using debug   = Mcu::Stm::Uart<1, void, 128, char, clock, MCU>;

    static inline void init() {
        clock::init();
        systemTimer::init();

        gpioa::init();
        gpiob::init();
        // gpioc::init();

        debug::init();
        debug::baud(115'200);
        // debug::halfDuplex();
        debugtx::afunction(0);


        crsf::init();
        crsf::baud(420'000);
        crsf::halfDuplex();
        crsftx::afunction(1);
        crsftx::template pullup<true>();

    }
};

template<template<typename, typename> typename CrsfCallback, typename MCU>
struct Devices2<SW12, CrsfCallback, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpioa, 2, MCU>;

    // Usart 2: CRSF
    struct CrsfAdapterConfig;
    using crsf_pa = RC::Protokoll::Crsf::Adapter<0, CrsfAdapterConfig, MCU>;
    using crsf    = Mcu::Stm::Uart<2, crsf_pa, 128, std::byte, clock, MCU>;
    using crsf_out= RC::Protokoll::Crsf::Generator<crsf, systemTimer, MCU>;

    using debugtx = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using debug   = Mcu::Stm::Uart<1, void, 1024, char, clock, MCU>;

    // Led
    using led = Mcu::Stm::Pin<gpioc, 15, MCU>;
    using ledBlinker = External::Blinker<led, systemTimer>;

    // Taster
    using button = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using btn = External::Button<button, systemTimer, External::Tick<systemTimer>{300ms}.raw(),
                                 External::Tick<systemTimer>{3000ms}.raw()>;

    using sw0 = Mcu::Stm::Pin<gpiob, 7, MCU>;
    using sw1 = Mcu::Stm::Pin<gpioa, 12, MCU>;
    using sw2 = Mcu::Stm::Pin<gpioa, 11, MCU>;
    using sw3 = Mcu::Stm::Pin<gpioa, 8, MCU>;
    using sw4 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using sw5 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using sw6 = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using sw7 = Mcu::Stm::Pin<gpioa, 4, MCU>;

    using pwm3 = Mcu::Stm::V2::Pwm::Simple<3, clock>;
    using pwm1 = Mcu::Stm::V2::Pwm::Simple<1, clock>;
    using pwm14 = Mcu::Stm::V2::Pwm::Simple<14, clock>;
    using pwm17 = Mcu::Stm::V2::Pwm::Simple<17, clock>;

    // s0 : pb7  : tim17 ch1n
    // s1 : pa12 : isr
    // s2 : pa11 : tim1 ch4
    // s3 : pb0 pa8  : tim1 ch1
    // s4 : pa7  : tim3 ch2
    // s5 : pa6  : tim3 ch1
    // s6 : pa5  : isr
    // s7 : pa4  : tim14 ch1

    using debug1 = void;
    using adap0 = Local::PwmAdapter<pwm17, 1, true, true, debug1>;
    using adap1 = Local::PwmAdapter<pwm3, 3, false, false, debug1>;
    using adap2 = Local::PwmAdapter<pwm1, 4, false, true, debug1>;
    using adap3 = Local::PwmAdapter<pwm1, 1, false, true, debug1>;
    using adap4 = Local::PwmAdapter<pwm3, 2, false, true, debug1>;
    using adap5 = Local::PwmAdapter<pwm3, 1, false, true, debug1>;
    using adap6 = Local::PwmAdapter<pwm3, 4, false, false, debug1>;
    using adap7 = Local::PwmAdapter<pwm14, 1, false, true, debug1>;

    using bsw0 = External::BlinkerWithPwm<sw0, systemTimer, adap0, debug1>;
    using bsw1 = External::BlinkerWithPwm<sw1, systemTimer, adap1, debug1>;
    using bsw2 = External::BlinkerWithPwm<sw2, systemTimer, adap2, debug1>;
    using bsw3 = External::BlinkerWithPwm<sw3, systemTimer, adap3, debug1>;
    using bsw4 = External::BlinkerWithPwm<sw4, systemTimer, adap4, debug1>;
    using bsw5 = External::BlinkerWithPwm<sw5, systemTimer, adap5, debug1>;
    using bsw6 = External::BlinkerWithPwm<sw6, systemTimer, adap6, debug1>;
    using bsw7 = External::BlinkerWithPwm<sw7, systemTimer, adap7, debug1>;

    using bsws = Meta::List<bsw0, bsw1, bsw2, bsw3, bsw4, bsw5, bsw6, bsw7>;

    struct CrsfCallbackConfig {
        using bswList = bsws;
        using pwmList = Meta::List<pwm1, pwm3, pwm14, pwm17>;
        using telem_out = crsf_out;
        using timer = systemTimer;
    };

    using crsfCallback = CrsfCallback<CrsfCallbackConfig, void>;

    struct CrsfAdapterConfig {
        using out = crsf_out;
        // using dbg = debug;
        using dbg = void;
        using callback = crsfCallback;
        using timer = systemTimer;
    };


    static inline void init() {
        CrsfAdapterConfig::callback::update();

        clock::init();
        systemTimer::init();

        gpioa::init();
        gpiob::init();
        gpioc::init();

        led::template dir<Mcu::Output>();
        ledBlinker::event(ledBlinker::Event::Off);

        button::template dir<Mcu::Input>();
        button::template pullup<true>();

        crsf::init();
        crsf::baud(420'000);
        crsf::halfDuplex();
        crsftx::afunction(1);
        crsftx::template pullup<true>();

        // if constexpr(!std::is_same_v<debug, void>) {
        //     debug::init();
        //     debug::baud(420'000);
        //     debugtx::afunction(0);
        // }

        sw0::template dir<Mcu::Output>();
        sw1::template dir<Mcu::Output>();
        sw2::template dir<Mcu::Output>();
        sw3::template dir<Mcu::Output>();
        sw4::template dir<Mcu::Output>();
        sw5::template dir<Mcu::Output>();
        sw6::template dir<Mcu::Output>();
        sw7::template dir<Mcu::Output>();

        pwm3::duty1(100);
        pwm3::duty2(100);
        pwm3::duty3(100);
        pwm3::duty4(100);
        pwm3::init();

        pwm3::template enableInts<adap1::channel>();
        pwm3::template enableInts<adap6::channel>();
        pwm1::init();
        pwm14::init();
        pwm17::init();

    }
};

template<template<typename, typename> typename CrsfCallback, typename MCU>
struct Devices2<SW11, CrsfCallback, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpioa, 2, MCU>;

    // Usart 2: CRSF
    struct CrsfAdapterConfig;
    using crsf_pa = RC::Protokoll::Crsf::Adapter<0, CrsfAdapterConfig, MCU>;
    using crsf    = Mcu::Stm::Uart<2, crsf_pa, RC::Protokoll::Crsf::maxMessageSize, std::byte, clock, MCU>;
    using crsf_out= RC::Protokoll::Crsf::Generator<crsf, systemTimer, MCU>;

    using debugtx = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using debug   = Mcu::Stm::Uart<1, void, 256, char, clock, MCU>;
    // using debug = void;

    // Led
    using led = Mcu::Stm::Pin<gpioc, 15, MCU>;
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

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;
    using pwmDmaCh_S1On = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    // using pwmDmaCh_S1Off = Mcu::Stm::Dma::Channel<dma1, 2, MCU>;
    // using pwmDmaCh_S6On = Mcu::Stm::Dma::Channel<dma1, 3, MCU>;
    // using pwmDmaCh_S6Off = Mcu::Stm::Dma::Channel<dma1, 4, MCU>;

    using sw0 = Mcu::Stm::Pin<gpiob, 7, MCU>;
    using sw1 = Mcu::Stm::Pin<gpioa, 12, MCU>;
    using sw2 = Mcu::Stm::Pin<gpioa, 11, MCU>;
    using sw3 = Mcu::Stm::Pin<gpioa, 8, MCU>;
    using sw4 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using sw5 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using sw6 = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using sw7 = Mcu::Stm::Pin<gpioa, 4, MCU>;

    using pwm3 = Mcu::Stm::V2::Pwm::Simple<3, clock>;
    using pwm1 = Mcu::Stm::V2::Pwm::Simple<1, clock>;
    using pwm14 = Mcu::Stm::V2::Pwm::Simple<14, clock>;
    using pwm17 = Mcu::Stm::V2::Pwm::Simple<17, clock>;


    // dma to gpio is not possible  !!!!!!!!!!!!!!!!!

    // using sw1dmaOff = Local::PinDma<sw1, Local::Reset, pwmDmaCh_S1Off, 37>; // update
    using sw1dmaOn  = External::PinDma<sw1, External::Set, pwmDmaCh_S1On, 34>; // ch3
    // using sw6dmaOff = Local::PinDma<sw6, Local::Reset, pwmDmaCh_S6Off, 37>;
    // using sw6dmaOn  = Local::PinDma<sw6, Local::Set, pwmDmaCh_S6On, 35>; // ch4


    // s0 : pb7  : tim17 ch1n
    // s1 : pa12 : isr
    // s2 : pa11 : tim1 ch4
    // s3 : pb0 pa8  : tim1 ch1
    // s4 : pa7  : tim3 ch2
    // s5 : pa6  : tim3 ch1
    // s6 : pa5  : isr
    // s7 : pa4  : tim14 ch1

    using debug1 = void;
    using adap0 = Local::PwmAdapter<pwm17, 1, true, true, debug1>;
    using adap1 = Local::PwmAdapter<pwm3, 3, false, false, debug>;
    using adap2 = Local::PwmAdapter<pwm1, 4, false, true, debug1>;
    using adap3 = Local::PwmAdapter<pwm1, 1, false, true, debug1>;
    using adap4 = Local::PwmAdapter<pwm3, 2, false, true, debug1>;
    using adap5 = Local::PwmAdapter<pwm3, 1, false, true, debug1>;
    using adap6 = Local::PwmAdapter<pwm3, 4, false, false, debug1>;
    using adap7 = Local::PwmAdapter<pwm14, 1, false, true, debug1>;

    using bsw0 = External::BlinkerWithPwm<sw0, systemTimer, adap0, debug1>;
    using bsw1 = External::BlinkerWithPwm<sw1, systemTimer, adap1, debug>;
    using bsw2 = External::BlinkerWithPwm<sw2, systemTimer, adap2, debug1>;
    using bsw3 = External::BlinkerWithPwm<sw3, systemTimer, adap3, debug1>;
    using bsw4 = External::BlinkerWithPwm<sw4, systemTimer, adap4, debug1>;
    using bsw5 = External::BlinkerWithPwm<sw5, systemTimer, adap5, debug1>;
    using bsw6 = External::BlinkerWithPwm<sw6, systemTimer, adap6, debug1>;
    using bsw7 = External::BlinkerWithPwm<sw7, systemTimer, adap7, debug1>;

    using bsws = Meta::List<bsw0, bsw1, bsw2, bsw3, bsw4, bsw5, bsw6, bsw7>;

    struct CrsfAdapterConfig {
        using out = crsf_out;
        // using dbg = debug;
        using dbg = void;
        using callback = CrsfCallback<bsws, debug>;
    };


    static inline void init() {
        CrsfAdapterConfig::callback::update();

        clock::init();
        systemTimer::init();

        gpioa::init();
        gpiob::init();
        gpioc::init();

        led::template dir<Mcu::Output>();

        crsf::init();
        crsf::baud(420'000);
        crsf::halfDuplex();
        crsftx::afunction(1);
        crsftx::template pullup<true>();

        // if constexpr(!std::is_same_v<debug, void>) {
        //     debug::init();
        //     debug::baud(420'000);
        //     debugtx::afunction(0);
        // }

        sw0::template dir<Mcu::Output>();
        sw1::template dir<Mcu::Output>();
        sw2::template dir<Mcu::Output>();
        sw3::template dir<Mcu::Output>();
        sw4::template dir<Mcu::Output>();
        sw5::template dir<Mcu::Output>();
        sw6::template dir<Mcu::Output>();
        sw7::template dir<Mcu::Output>();

        // sw1dmaOff::init();
        sw1dmaOn::init();
        // sw6dmaOff::init();
        // sw6dmaOn::init();

        pwm3::duty1(100);
        pwm3::duty2(100);
        pwm3::duty3(100);
        pwm3::duty4(100);
        pwm3::init();

        pwm3::template enableDma<adap1::channel>();
        // pwm3::template enableDma<adap6::channel>();
        pwm1::init();
        pwm14::init();
        pwm17::init();

    }
};

template<template<typename, typename> typename CrsfCallback, typename MCU>
struct Devices2<SW10, CrsfCallback, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpioa, 2, MCU>;

    // Usart 2: CRSF
    struct CrsfAdapterConfig;
    using crsf_pa = RC::Protokoll::Crsf::Adapter<0, CrsfAdapterConfig, MCU>;
    using crsf    = Mcu::Stm::Uart<2, crsf_pa, RC::Protokoll::Crsf::maxMessageSize, std::byte, clock, MCU>;
    using crsf_out= RC::Protokoll::Crsf::Generator<crsf, systemTimer, MCU>;

    using debugtx = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using debug   = Mcu::Stm::Uart<1, void, 256, char, clock, MCU>;
    // using debug = void;

    // Led
    using led = Mcu::Stm::Pin<gpioc, 15, MCU>;
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

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;
    using pwmDmaCh_S1On = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using pwmDmaCh_S1Off = Mcu::Stm::Dma::Channel<dma1, 2, MCU>;
    using pwmDmaCh_S6On = Mcu::Stm::Dma::Channel<dma1, 3, MCU>;
    using pwmDmaCh_S6Off = Mcu::Stm::Dma::Channel<dma1, 4, MCU>;

    using sw0 = Mcu::Stm::Pin<gpiob, 7, MCU>;
    using sw1 = Mcu::Stm::Pin<gpioa, 12, MCU>;
    using sw2 = Mcu::Stm::Pin<gpioa, 11, MCU>;
    // using sw3 = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using sw3 = Mcu::Stm::Pin<gpioa, 8, MCU>;
    using sw4 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using sw5 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using sw6 = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using sw7 = Mcu::Stm::Pin<gpioa, 4, MCU>;

    using pwm3 = Mcu::Stm::V2::Pwm::Simple<3, clock>;
    using pwm1 = Mcu::Stm::V2::Pwm::Simple<1, clock>;
    using pwm14 = Mcu::Stm::V2::Pwm::Simple<14, clock>;
    using pwm17 = Mcu::Stm::V2::Pwm::Simple<17, clock>;

    // s0 : pb7  : tim17 ch1n
    // s1 : pa12 : isr
    // s2 : pa11 : tim1 ch4
    // s3 : pb0 pa8  : tim1 ch1
    // s4 : pa7  : tim3 ch2
    // s5 : pa6  : tim3 ch1
    // s6 : pa5  : isr
    // s7 : pa4  : tim14 ch1

    using debug1 = void;
    using adap0 = Local::PwmAdapter<pwm17, 1, true, true, debug1>;
    using adap1 = Local::PwmAdapter<pwm3, 3, false, false, debug1>;
    using adap2 = Local::PwmAdapter<pwm1, 4, false, true, debug1>;
    using adap3 = Local::PwmAdapter<pwm1, 1, false, true, debug1>;
    using adap4 = Local::PwmAdapter<pwm3, 2, false, true, debug1>;
    using adap5 = Local::PwmAdapter<pwm3, 1, false, true, debug1>;
    using adap6 = Local::PwmAdapter<pwm3, 4, false, false, debug1>;
    using adap7 = Local::PwmAdapter<pwm14, 1, false, true, debug1>;

    using bsw0 = External::BlinkerWithPwm<sw0, systemTimer, adap0, debug1>;
    using bsw1 = External::BlinkerWithPwm<sw1, systemTimer, adap1, debug1>;
    using bsw2 = External::BlinkerWithPwm<sw2, systemTimer, adap2, debug1>;
    using bsw3 = External::BlinkerWithPwm<sw3, systemTimer, adap3, debug1>;
    using bsw4 = External::BlinkerWithPwm<sw4, systemTimer, adap4, debug1>;
    using bsw5 = External::BlinkerWithPwm<sw5, systemTimer, adap5, debug1>;
    using bsw6 = External::BlinkerWithPwm<sw6, systemTimer, adap6, debug1>;
    using bsw7 = External::BlinkerWithPwm<sw7, systemTimer, adap7, debug1>;

    using bsws = Meta::List<bsw0, bsw1, bsw2, bsw3, bsw4, bsw5, bsw6, bsw7>;

    struct CrsfAdapterConfig {
        using out = crsf_out;
        // using dbg = debug;
        using dbg = void;
        using callback = CrsfCallback<bsws, debug>;
    };


    static inline void init() {
        CrsfAdapterConfig::callback::update();

        clock::init();
        systemTimer::init();

        gpioa::init();
        gpiob::init();
        gpioc::init();

        led::template dir<Mcu::Output>();

        crsf::init();
        crsf::baud(420'000);
        crsf::halfDuplex();
        crsftx::afunction(1);
        crsftx::template pullup<true>();

        // if constexpr(!std::is_same_v<debug, void>) {
        //     debug::init();
        //     debug::baud(420'000);
        //     debugtx::afunction(0);
        // }

        sw0::template dir<Mcu::Output>();
        sw1::template dir<Mcu::Output>();
        sw2::template dir<Mcu::Output>();
        sw3::template dir<Mcu::Output>();
        sw4::template dir<Mcu::Output>();
        sw5::template dir<Mcu::Output>();
        sw6::template dir<Mcu::Output>();
        sw7::template dir<Mcu::Output>();

        pwm3::init();
        pwm3::template enableInts<adap1::channel>();
        pwm3::template enableInts<adap6::channel>();
        pwm1::init();
        pwm14::init();
        pwm17::init();
    }
};

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
    using pwm1 = Mcu::Stm::V2::Pwm::Simple<1, clock>;
    using pwm17 = Mcu::Stm::V2::Pwm::Simple<17, clock>;
    using pwm14 = Mcu::Stm::V2::Pwm::Simple<14, clock>;

    // s0 : pb7  : tim17 ch1n
    // s1 : pa12 : isr
    // s2 : pa11 : tim1 ch4
    // s3 : pb0  : tim1 ch1
    // s4 : pa7  : tim3 ch2
    // s5 : pa6  : tim3 ch1
    // s6 : pa5  : isr
    // s7 : pa4  : tim14 ch1

    using debug1 = void;
    using adap2 = Local::PwmAdapter<pwm1, 3, false, true, debug1>;
    using adap3 = Local::PwmAdapter<pwm1, 0, false, true, debug1>;
    using adap4 = Local::PwmAdapter<pwm3, 1, false, true, debug1>;
    using adap5 = Local::PwmAdapter<pwm3, 0, false, true, debug1>;
    using adap7 = Local::PwmAdapter<pwm14, 0, false, true, debug1>;

    using bsw0 = External::BlinkerWithPwm<sw0, systemTimer, void, debug1>;
    using bsw1 = External::BlinkerWithPwm<sw1, systemTimer, void, debug1>;
    using bsw2 = External::BlinkerWithPwm<sw2, systemTimer, adap2, debug1>;
    using bsw3 = External::BlinkerWithPwm<sw3, systemTimer, adap3, debug1>;
    using bsw4 = External::BlinkerWithPwm<sw4, systemTimer, adap4, debug1>;
    using bsw5 = External::BlinkerWithPwm<sw5, systemTimer, adap5, debug1>;
    using bsw6 = External::BlinkerWithPwm<sw6, systemTimer, void, debug1>;
    using bsw7 = External::BlinkerWithPwm<sw7, systemTimer, adap7, debug1>;

    using bsws = Meta::List<bsw0, bsw1, bsw2, bsw3, bsw4, bsw5, bsw6, bsw7>;

    struct CrsfAdapterConfig {
        using out = crsf_out;
        using callback = CrsfCallback<bsws, void>;
        using dbg = debug;
    };

    static inline void init() {
        CrsfAdapterConfig::callback::update();

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
        pwm1::init();
        pwm14::init();
        pwm17::init();
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
        using dbg = debug;
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
        // pwm3::template enableDma<3>();
        // pwm3::template enableDma<4>();
    }
};

