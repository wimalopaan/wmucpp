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
#include "rc/rc_2.h"
#include "rc/crsf_2.h"
#include "blinker.h"
#include "debug_2.h"

#include "adapter.h"
#include "button.h"

struct Nucleo;
struct SW01;
struct SW10; // board: RCMultiSwitchSmall10
struct SW11; // board: RCMultiSwitchSmall10 // wrong dma (does not work with GPIO on G0xx!)
struct SW12; // board: RCMultiSwitchSmall10
struct SW13; // board: RCMultiSwitchSmall10

struct SW20; // board: RCMultiSwitchSmall10

struct SW99; // test

using namespace std::literals::chrono_literals;

template<typename HW, template<typename, typename> typename CrsfCallback, typename MCU = DefaultMcu>
struct Devices2;

template<template<typename, typename> typename CrsfCallback, typename MCU>
struct Devices2<SW20, CrsfCallback, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpioa, 2, MCU>;

    // Usart 2: CRSF
    using csrfInDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
    struct CrsfConfig;
    using crsf = RC::Protokoll::Crsf::V4::Master<2, CrsfConfig, MCU>;

#ifdef SERIAL_DEBUG
    using debugtx = Mcu::Stm::Pin<gpiob, 6, MCU>;
    struct DebugConfig;
    using debug = SerialBuffered<1, DebugConfig, MCU>;
    struct DebugConfig {
        using pin = debugtx;
        using clock = Devices2::clock;
        static inline constexpr bool rxtxswap = false;
        static inline constexpr uint16_t bufferSize = 128;
    };
#else
    using debug = void;
#endif

    // Led
    using led = Mcu::Stm::Pin<gpioc, 15, MCU>;
    using ledBlinker = External::Blinker<led, systemTimer>;

    // Taster
#ifdef USE_BUTTON
    using button = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using btn = External::Button<button, systemTimer, External::Tick<systemTimer>{300ms}.raw(),
                                 External::Tick<systemTimer>{3000ms}.raw(), debug>;
#endif
#ifdef USE_TP1
    using tp1 = Mcu::Stm::Pin<gpioa, 3, MCU>;
#else
    using tp1 = void;
#endif
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
        using timer = systemTimer;
        using crsf = Devices2::crsf;
    };
    struct CrsfConfig {
        using txpin = crsftx;
        using rxpin = txpin; // half-duplex
        using systemTimer = Devices2::systemTimer;
        using clock = Devices2::clock;
        using dmaChRW  = csrfInDmaChannelComponent1;
        // using debug = void;
        using debug = Devices2::debug;
        using tp = tp1;
        using callback = CrsfCallback<CrsfCallbackConfig, debug>;
        // using callback = CrsfCallback<CrsfCallbackConfig, void>;
        static inline constexpr uint8_t fifoSize = 8;
    };
    static inline void init() {
        clock::init();
        systemTimer::init();

        gpioa::init();
        gpiob::init();
        gpioc::init();

        dma1::init();

        led::template dir<Mcu::Output>();
        ledBlinker::event(ledBlinker::Event::Off);

#ifdef USE_BUTTON
        btn::init();
#endif
#ifdef USE_TP1
        tp1::template dir<Mcu::Output>();
#endif
        crsf::init();

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

