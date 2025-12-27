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
#include "rc/router.h"

struct SW01;
struct SW10;

template<typename HW, typename Config, typename MCU = DefaultMcu>
struct Devices;

template<typename Config, typename MCU>
struct Devices<SW10, Config, MCU> {
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
    
	using csrfFd3DmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 5>;
	using csrfFd3DmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma1::component_t, 6>;
	
    using csrfFd4DmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 7>;
	using csrfFd4DmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma2::component_t, 1>;
  
	using csrfFd5DmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma2::component_t, 2>;
	using csrfFd5DmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma2::component_t, 3>;
	
    using csrfFd6DmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma2::component_t, 4>;
    using csrfFd6DmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma2::component_t, 5>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpioa, 9, MCU>;
    using crsfrx = Mcu::Stm::Pin<gpioa, 10, MCU>;

    // Usart 1: CRSF
    struct CrsfInConfig;
    using crsf_in = RC::Protokoll::Crsf::V4::Master<1, CrsfInConfig, MCU>;
    using crsf_in_buffer = crsf_in::messageBuffer;

    // LPUART 1: CRSF HD2
    using crsf_hd1_rxtx = Mcu::Stm::Pin<gpioa, 2, MCU>;
    struct CrsfHd1Config;
    using crsf_1 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<101, CrsfHd1Config, MCU>;

    // USART 2: CRSF HD1
    using crsf_hd2_rxtx = Mcu::Stm::Pin<gpioa, 15, MCU>;
    struct CrsfHd2Config;
    using crsf_2 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<2, CrsfHd2Config, MCU>;

    // USART 3: CRSF FD1
    using crsf_fd3_tx = Mcu::Stm::Pin<gpiob, 8, MCU>;
	using crsf_fd3_rx = Mcu::Stm::Pin<gpiob, 9, MCU>;
    struct CrsfFD3Config;
    using crsf_3 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<3, CrsfFD3Config, MCU>;

	// USART 4: CRSF FD2
    using crsf_fd4_tx = Mcu::Stm::Pin<gpioa, 0, MCU>;
	using crsf_fd4_rx = Mcu::Stm::Pin<gpioa, 1, MCU>;
#ifndef USE_DEBUG
	struct CrsfFD4Config;
    using crsf_4 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<4, CrsfFD4Config, MCU>;
	using debug = void;
#else
    using crsf_4 = struct Dummy {
        static inline void init(){}
        static inline void periodic(){}
        static inline void ratePeriodic(){}
        static inline void baud(auto) {}
        static inline void forwardPacket(auto, auto) {}
        static inline void enable(const bool) {}
        static inline void activateRouter(const bool){}
        static inline void activateSource(const bool) {}
        static inline void txAddress(const uint8_t) {}
        static inline void rxAddress(const uint8_t) {}
        static inline void activateLinkStats(const bool) {}
        static inline void activateChannels(const bool) {}
        static inline void activateBroadcast(const bool) {}
        static inline void telemetryRate(const uint8_t) {}
        static inline void insertRoute(const uint8_t) {}
        static inline void rewriteName(auto) {}
        static inline void tunnelLinkStat(auto) {}
        static inline void forwardTelemetry(auto) {}
        static inline void tunnelTelemetry(auto) {}
    };
	using debugtx = crsf_fd4_tx;
    struct DebugConfig;
    using debug = SerialBuffered<4, DebugConfig, MCU>;
    struct DebugConfig {
        using pin = debugtx;
        using clock = Devices::clock;
        static inline constexpr uint16_t bufferSize = 512;
    };
#endif

    // USART 5: CRSF FD3
    using crsf_fd5_tx = Mcu::Stm::Pin<gpiob, 0, MCU>;
	using crsf_fd5_rx = Mcu::Stm::Pin<gpiob, 1, MCU>;
    struct CrsfFD5Config;
    using crsf_5 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<5, CrsfFD5Config, MCU>;

    // USART 6: CRSF FD4
    using crsf_hd6_tx = Mcu::Stm::Pin<gpioa, 4, MCU>;
	using crsf_hd6_rx = Mcu::Stm::Pin<gpioa, 5, MCU>;
    struct CrsfFD6Config;
    using crsf_6 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<6, CrsfFD6Config, MCU>;

#ifdef USE_DEBUG
    // using crsf_ifaces = Meta::List<crsf_1, crsf_2, crsf_3, crsf_5, crsf_6>;
    using crsf_ifaces = Meta::List<crsf_2, crsf_1, crsf_3, crsf_4, crsf_5, crsf_6>;
#else
    using crsf_ifaces = Meta::List<crsf_2, crsf_1, crsf_3, crsf_4, crsf_5, crsf_6>;
#endif
	
    // Led
    using led = Mcu::Stm::Pin<gpiob, 2, MCU>;
    using ledBlinker = External::Blinker<led, systemTimer>;

    template<auto N>
    struct StorageAdapter {
        static inline struct E {
            static inline auto& address = Devices::storage::eeprom.address;
            static inline auto& commandBroadcastAddress = Devices::storage::eeprom.commandBroadcastAddress;
            static inline auto& txname  = Devices::storage::eeprom.outputParams[N].tx_name;
        } eeprom;
    };

    struct RouterConfig;
    using router = Router<RouterConfig>;
    struct RouterConfig {
        using storage = Devices::storage;
        using debug = Devices::debug;
        // using debug = void;
    };

    struct CrsfInCallbackConfig {
        using debug = void;
        // using debug = Devices::debug;
        using timer = systemTimer;
        using crsf = Devices::crsf_in;
        using storage = Devices::storage;
        using crsf_ifaces = Devices::crsf_ifaces;
        using src = Devices::crsf_in;
    };
    using crsf_cb = CrsfCallback<CrsfInCallbackConfig>;
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
        using callback = crsf_cb;
        static inline constexpr uint8_t fifoSize = 8;
    };
    using debug1 = void;
    struct CrsfHd1Config {
        using router = Devices::router;
		using bcastInterfaces = Meta::List<crsf_2, crsf_3, crsf_4, crsf_5, crsf_6>;
        static inline constexpr uint8_t id = 1;
        using src = crsf_in::input;
        using dest = crsf_in::messageBuffer;
        using rxpin = crsf_hd1_rxtx;
        using txpin = crsf_hd1_rxtx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfHd1DmaChannelComponent;
        // using storage = Devices::storage;
        using storage =  StorageAdapter<0>;
        using debug = Devices::debug1;
        using tp = void;
        using tp1 = void;
        using tp2 = void;
        static inline constexpr uint8_t fifoSize = 16;
    };
    struct CrsfHd2Config {
        using router = Devices::router;
		using bcastInterfaces = Meta::List<crsf_1, crsf_3, crsf_4, crsf_5, crsf_6>;
        static inline constexpr uint8_t id = 2;
        using src = crsf_in::input;
        using dest = crsf_in::messageBuffer;
		static inline constexpr bool rxtxswap = true;
        using rxpin = crsf_hd2_rxtx;
        using txpin = crsf_hd2_rxtx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfHd2DmaChannelComponent;
        // using storage = Devices::storage;
        using storage =  StorageAdapter<1>;
        using debug = Devices::debug1;
        using tp = void;
        using tp1 = void;
        using tp2 = void;
        static inline constexpr uint8_t fifoSize = 16;
    };
    struct CrsfFD3Config {
        using router = Devices::router;
		using bcastInterfaces = Meta::List<crsf_1, crsf_2, crsf_4, crsf_5, crsf_6>;
        static inline constexpr uint8_t id = 3;
        using src = crsf_in::input;
        using dest = crsf_in::messageBuffer;
        using rxpin = crsf_fd3_rx;
        using txpin = crsf_fd3_tx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfFd3DmaChannelComponent1;
		using dmaChWrite = csrfFd3DmaChannelComponent2;
        // using storage = Devices::storage;
        using storage =  StorageAdapter<2>;
        using debug = Devices::debug1;
        using tp = void;
        using tp1 = void;
        using tp2 = void;
        static inline constexpr uint8_t fifoSize = 16;
    };
	struct CrsfFD4Config {
        using router = Devices::router;
		using bcastInterfaces = Meta::List<crsf_1, crsf_2, crsf_3, crsf_5, crsf_6>;
        static inline constexpr uint8_t id = 4;
        using src = crsf_in::input;
        using dest = crsf_in::messageBuffer;
        using rxpin = crsf_fd4_rx;
        using txpin = crsf_fd4_tx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfFd4DmaChannelComponent1;
		using dmaChWrite = csrfFd4DmaChannelComponent2;
        // using storage = Devices::storage;
        using storage =  StorageAdapter<3>;
        using debug = Devices::debug1;
        using tp = void;
        using tp1 = void;
        using tp2 = void;
        static inline constexpr uint8_t fifoSize = 16;
    };
    struct CrsfFD5Config {
        using router = Devices::router;
		using bcastInterfaces = Meta::List<crsf_1, crsf_2, crsf_3, crsf_4, crsf_6>;
        static inline constexpr uint8_t id = 5;
        using src = crsf_in::input;
        using dest = crsf_in::messageBuffer;
        using rxpin = crsf_fd5_rx;
        using txpin = crsf_fd5_tx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfFd5DmaChannelComponent1;
		using dmaChWrite = csrfFd5DmaChannelComponent2;
        // using storage = Devices::storage;
        using storage =  StorageAdapter<4>;
        using debug = Devices::debug;
        using tp = void;
        using tp1 = void;
        using tp2 = void;
        static inline constexpr uint8_t fifoSize = 16;
    };
    struct CrsfFD6Config {
        using router = Devices::router;
		using bcastInterfaces = Meta::List<crsf_1, crsf_2, crsf_3, crsf_4, crsf_5>;
        static inline constexpr uint8_t id = 6;
        using src = crsf_in::input;
        using dest = crsf_in::messageBuffer;
        using rxpin = crsf_hd6_rx;
        using txpin = crsf_hd6_tx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfFd6DmaChannelComponent1;
		using dmaChWrite  = csrfFd6DmaChannelComponent2;
        // using storage = Devices::storage;
        using storage =  StorageAdapter<5>;
        using debug = Devices::debug1;
        using tp = void;
        using tp1 = void;
        using tp2 = void;
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

        crsf_in::init();
        crsf_in::baud(420'000);

        crsf_1::init();
        crsf_2::init();
        crsf_3::init();
#ifdef USE_DEBUG
        debug::init();
#else
		crsf_4::init();
#endif
        crsf_5::init();
        crsf_6::init();
    }
};

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
    using csrfHd6DmaChannelComponent = Mcu::Components::DmaChannel<typename dma2::component_t, 1>;
    using csrfHd7DmaChannelComponent = Mcu::Components::DmaChannel<typename dma2::component_t, 2>;

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
    using crsf_1 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<101, CrsfHd1Config, MCU>;

    // USART 2: CRSF HD2
    using crsf_hd2_rxtx = Mcu::Stm::Pin<gpioa, 14, MCU>;
    struct CrsfHd2Config;
    using crsf_2 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<2, CrsfHd2Config, MCU>;

    // USART 3: CRSF HD3
    using crsf_hd3_rxtx = Mcu::Stm::Pin<gpioa, 5, MCU>;
    struct CrsfFD3Config;
    using crsf_3 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<3, CrsfFD3Config, MCU>;

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
    using crsf_5 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<5, CrsfHd5Config, MCU>;

    // USART 6: CRSF HD6
    using crsf_hd6_rxtx = Mcu::Stm::Pin<gpioa, 4, MCU>;
    struct CrsfHd6Config;
    using crsf_6 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<6, CrsfHd6Config, MCU>;

    // LPUART 2: CRSF HD7
    using crsf_hd7_rxtx = Mcu::Stm::Pin<gpiob, 6, MCU>;
    struct CrsfHd7Config;
    using crsf_7 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<102, CrsfHd7Config, MCU>;

    using crsf_ifaces = Meta::List<crsf_1, crsf_2, crsf_3, crsf_5, crsf_6, crsf_7>;

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
        using crsf_ifaces = Devices::crsf_ifaces;
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
    struct CrsfFD3Config {
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
    struct CrsfHd6Config {
        using router = Devices::router;
        static inline constexpr uint8_t id = 6;
        using src = crsf_in::input;
        using dest = crsf_in::messageBuffer;
        using rxpin = crsf_hd6_rxtx;
        using txpin = crsf_hd6_rxtx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfHd6DmaChannelComponent;
        using storage = Devices::storage;
        using debug = Devices::debug1;
        using tp = void;
        static inline constexpr uint8_t fifoSize = 16;
    };
    struct CrsfHd7Config {
        using router = Devices::router;
        static inline constexpr uint8_t id = 7;
        using src = crsf_in::input;
        using dest = crsf_in::messageBuffer;
        using rxpin = crsf_hd7_rxtx;
        using txpin = crsf_hd7_rxtx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfHd7DmaChannelComponent;
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

        crsf_1::init();
        crsf_1::baud(420'000);

        crsf_2::init();
        crsf_2::baud(420'000);

        crsf_3::init();
        crsf_3::baud(420'000);

        crsf_5::init();
        crsf_5::baud(420'000);

        crsf_6::init();
        crsf_6::baud(420'000);

        crsf_7::init();
        crsf_7::baud(420'000);
    }
};


