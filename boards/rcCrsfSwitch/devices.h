/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "meta.h"
#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "components.h"
#include "usart_2.h"
#include "units.h"
#include "output.h"
#include "debug_2.h"
#include "concepts.h"
#include "clock.h"
#include "gpio.h"
#include "tick.h"
#include "meta.h"
#include "blinker.h"

#include "rc/rc_2.h"
#include "rc/crsf_2.h"
#include "rc/package_relay_rewrite.h"

#include "crsfcb.h"
#include "router.h"

struct SW01;

template<typename HW, typename Config, typename MCU = DefaultMcu>
struct Devices;

template<typename Config, typename MCU>
struct Devices<SW01, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using storage = Config::storage;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;
    using dma2 = Mcu::Stm::Dma::Controller<2, MCU>;

    using csrfInDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
    using csrfInDmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma1::component_t, 2>;

    using csrfHd1DmaChannelComponent = Mcu::Components::DmaChannel<typename dma1::component_t, 3>;
    using csrfHd2DmaChannelComponent = Mcu::Components::DmaChannel<typename dma1::component_t, 4>;
    using csrfHd3DmaChannelComponent = Mcu::Components::DmaChannel<typename dma1::component_t, 5>;
    using csrfHd4DmaChannelComponent = Mcu::Components::DmaChannel<typename dma1::component_t, 6>;
    using csrfHd5DmaChannelComponent = Mcu::Components::DmaChannel<typename dma1::component_t, 7>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpioa, 9, MCU>;
    using crsfrx = Mcu::Stm::Pin<gpioa, 10, MCU>;

    // Usart 1: CRSF
    struct CrsfInConfig;
    using crsf_in = RC::Protokoll::Crsf::V4::Master<1, CrsfInConfig, MCU>;
    using crsf_in_buffer = crsf_in::messageBuffer;

    // LPUART 1: CRSF HD1
    using crsf_hd1_rxtx = Mcu::Stm::Pin<gpioa, 2, MCU>;
    struct CrsfHd1Config;
    using crsf_hd1 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<101, CrsfHd1Config, MCU>;

    // USART 2: CRSF HD2
    using crsf_hd2_rxtx = Mcu::Stm::Pin<gpioa, 14, MCU>;
    struct CrsfHd2Config;
    using crsf_hd2 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<2, CrsfHd2Config, MCU>;

    // USART 3: CRSF HD3
    using crsf_hd3_rxtx = Mcu::Stm::Pin<gpioa, 5, MCU>;
    struct CrsfHd3Config;
    using crsf_hd3 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<3, CrsfHd3Config, MCU>;

    // USART 4: CRSF HD4
    using crsf_hd4_rxtx = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using debugtx = crsf_hd4_rxtx;
    struct DebugConfig;
    using debug = SerialBuffered<4, DebugConfig, MCU>;
    struct DebugConfig {
        using pin = debugtx;
        using clock = Devices::clock;
        static inline constexpr uint16_t bufferSize = 512;
    };

    // USART 5: CRSF HD5
    using crsf_hd5_rxtx = Mcu::Stm::Pin<gpiob, 0, MCU>;
    struct CrsfHd5Config;
    using crsf_hd5 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<5, CrsfHd5Config, MCU>;

    // USART 6: CRSF HD6
    using crsf_hd6_rxtx = Mcu::Stm::Pin<gpioa, 4, MCU>;

    // LPUART 2: CRSF HD7
    using crsf_hd7_rxtx = Mcu::Stm::Pin<gpiob, 6, MCU>;

    using crsf_ifaces = Meta::List<crsf_hd1, crsf_hd2, crsf_hd3, crsf_hd5>;

    // Led
    using led = Mcu::Stm::Pin<gpiob, 9, MCU>;
    using ledBlinker = External::Blinker<led, systemTimer>;

    struct RouterConfig;
    using router = Router<RouterConfig>;
    struct RouterConfig {
        using storage = Devices::storage;
        // using debug = Devices::debug;
        using debug = void;
    };

    struct CrsfInCallbackConfig {
        using debug = void;
        // using debug = Devices::debug;
        using timer = systemTimer;
        using crsf = Devices::crsf_in;
        using storage = Devices::storage;
        using crsf_hd1 = Devices::crsf_hd1;
        using crsf_hd2 = Devices::crsf_hd2;
        using crsf_hd3 = Devices::crsf_hd3;
        using crsf_hd5 = Devices::crsf_hd5;
    };
    struct CrsfInConfig {
        using txpin = crsftx;
        using rxpin = crsfrx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfInDmaChannelComponent1;
        using dmaChWrite  = csrfInDmaChannelComponent2;
        using debug = void;
        // using debug = Devices::debug;
        using tp = void;
        using callback = CrsfCallback<CrsfInCallbackConfig>;
        static inline constexpr uint8_t fifoSize = 8;
    };
    using debug1 = void;
    struct CrsfHd1Config {
        using router = Devices::router;
        static inline constexpr uint8_t id = 1;
        using src = crsf_in::input;
        using dest = crsf_in::messageBuffer;
        using rxpin = crsf_hd1_rxtx;
        using txpin = crsf_hd1_rxtx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfHd1DmaChannelComponent;
        using storage = Devices::storage;
        using debug = Devices::debug1;
        using tp = void;
        static inline constexpr uint8_t fifoSize = 16;
    };
    struct CrsfHd2Config {
        using router = Devices::router;
        static inline constexpr uint8_t id = 2;
        using src = crsf_in::input;
        using dest = crsf_in::messageBuffer;
        using rxpin = crsf_hd2_rxtx;
        using txpin = crsf_hd2_rxtx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfHd2DmaChannelComponent;
        using storage = Devices::storage;
        using debug = Devices::debug1;
        using tp = void;
        static inline constexpr uint8_t fifoSize = 16;
    };
    struct CrsfHd3Config {
        using router = Devices::router;
        static inline constexpr uint8_t id = 3;
        using src = crsf_in::input;
        using dest = crsf_in::messageBuffer;
        using rxpin = crsf_hd3_rxtx;
        using txpin = crsf_hd3_rxtx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfHd3DmaChannelComponent;
        using storage = Devices::storage;
        using debug = Devices::debug1;
        using tp = void;
        static inline constexpr uint8_t fifoSize = 16;
    };
    struct CrsfHd5Config {
        using router = Devices::router;
        static inline constexpr uint8_t id = 5;
        using src = crsf_in::input;
        using dest = crsf_in::messageBuffer;
        using rxpin = crsf_hd5_rxtx;
        using txpin = crsf_hd5_rxtx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfHd5DmaChannelComponent;
        using storage = Devices::storage;
        using debug = Devices::debug1;
        using tp = void;
        static inline constexpr uint8_t fifoSize = 16;
    };

    static inline void init() {
        clock::init();
        systemTimer::init();

        dma1::init();
        dma2::init();

        gpioa::init();
        gpiob::init();
        gpioc::init();

        led::template dir<Mcu::Output>();

        debug::init();

        crsf_in::init();
        crsf_in::baud(420'000);

        crsf_hd1::init();
        crsf_hd1::baud(420'000);

        crsf_hd2::init();
        crsf_hd2::baud(420'000);

        crsf_hd3::init();
        crsf_hd3::baud(420'000);

        crsf_hd5::init();
        crsf_hd5::baud(420'000);
    }
};


