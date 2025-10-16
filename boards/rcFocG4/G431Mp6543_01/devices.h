/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "crsf_cb.h"

struct Foc01;

using namespace std::literals::chrono_literals;

template<typename HW, typename Config, typename MCU = DefaultMcu>
struct Devices;

template<typename Config, typename MCU>
struct Devices<Foc01, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

#ifdef USE_GNUPLOT
    using trace = Arm::Trace<clock, 10_MHz, 4096>;
#else
    using trace = Arm::V3::Trace<clock, 10_MHz, 1024>;
#endif

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    using storage = Config::storage;

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;
    using csrfInDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
    using spiDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 2>;
    // using spiDmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma1::component_t, 3>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpioa, 9, MCU>;

    // Usart 1: CRSF
    struct CrsfConfig;
    using crsf = RC::Protokoll::Crsf::V4::Master<1, CrsfConfig, MCU>;

#ifdef SERIAL_DEBUG
    // Usart 2: Debug
    using debugtx = Mcu::Stm::Pin<gpioa, 15, MCU>;
    struct DebugConfig;
    using debug = SerialBuffered<2, DebugConfig, MCU>;
    struct DebugConfig {
        using pin = debugtx;
        using clock = Devices::clock;
        static inline constexpr uint16_t bufferSize = 2048;
        static inline constexpr bool rxtxswap = true;
    };
#else
    using debug = void;
#endif

    // Led
    using led = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using ledBlinker = External::Blinker<led, systemTimer>;

    using tp = Mcu::Stm::Pin<gpioa, 4, MCU>;

    // using mco = Mcu::Stm::Pin<gpioa, 8, MCU>;

    // SPI 2:

    using spi_cs = Mcu::Stm::Pin<gpiof, 0, MCU>; // Tauschen mit ENA (PF0 -> PA12)
    using spi_miso = Mcu::Stm::Pin<gpioa, 10, MCU>;
    using spi_mosi = Mcu::Stm::Pin<gpioa, 11, MCU>;
    using spi_clk = Mcu::Stm::Pin<gpiof, 1, MCU>;

    using mps_fault = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using mps_enable = Mcu::Stm::Pin<gpioa, 12, MCU>;
    using mps_sleep = Mcu::Stm::Pin<gpiob, 4, MCU>;

    struct CrsfCallbackConfig {
        using timer = systemTimer;
        using crsf = Devices::crsf;
        using storage = Devices::storage;
    };
    struct CrsfConfig {
        using txpin = crsftx;
        using rxpin = crsftx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRW  = csrfInDmaChannelComponent1;
        // using debug = void;
        using debug = Devices::debug;
        using tp = tp;
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
        gpiof::init();

        dma1::init();

        led::template dir<Mcu::Output>();
        ledBlinker::event(ledBlinker::Event::Steady);

        tp::template dir<Mcu::Output>();

        // mco::afunction(0);

        crsf::init();
    }
};

