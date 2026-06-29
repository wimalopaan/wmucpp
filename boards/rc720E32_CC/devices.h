/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

struct CC10;

template<typename HW, typename Config, typename MCU = DefaultMcu>
struct Devices;

template<typename Config, typename MCU>
struct Devices<CC10, Config, MCU> {
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

    // USART 2: CRSF FD
    using crsf2_tx = Mcu::Stm::Pin<gpioa, 2, MCU>;
	using crsf2_rx = Mcu::Stm::Pin<gpioa, 3, MCU>;
    struct Crsf2Config;
    using crsf2 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<2, Crsf2Config, MCU>;

    // USART 3: CRSF FD
    using crsf3_tx = Mcu::Stm::Pin<gpiob, 2, MCU>;
	using crsf3_rx = Mcu::Stm::Pin<gpiob, 9, MCU>;
    struct Crsf3Config;
    using crsf3 = RC::Protokoll::Crsf::V4::PacketRelayRewrite<3, Crsf3Config, MCU>;
    
	// USART 4
    using sbus_tx = Mcu::Stm::Pin<gpioc, 6, MCU>;
    // using crsf4_tx = Mcu::Stm::Pin<gpioa, 0, MCU>;
	// using crsf4_rx = Mcu::Stm::Pin<gpioa, 1, MCU>;
	using debugtx = sbus_tx;
    struct DebugConfig;
    using debug = SerialBuffered<102, DebugConfig, MCU>;
    struct DebugConfig {
        using pin = debugtx;
        using clock = Devices::clock;
        static inline constexpr uint16_t bufferSize = 512;
    };

    using crsf_ifaces = Meta::List<crsf2, crsf3>;
	
    // Led
    // Led
    using led1 = Mcu::Stm::Pin<gpiob, 7, MCU>;
    using led2 = Mcu::Stm::Pin<gpiob, 8, MCU>;

    using ledBlinker1 = External::Blinker<led1, systemTimer>;
    using ledBlinker2 = External::Blinker<led2, systemTimer>;

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
    struct Crsf2Config {
        using router = Devices::router;
		using bcastInterfaces = Meta::List<crsf3>;
        static inline constexpr uint8_t id = 1;
        using src = crsf_in::input;
        using dest = crsf_in::messageBuffer;
        using rxpin = crsf2_rx;
        using txpin = crsf2_tx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfFd3DmaChannelComponent1;
		using dmaChWrite = csrfFd3DmaChannelComponent2;
        using storage =  StorageAdapter<0>;
        using debug = Devices::debug;
        using tp = void;
        using tp1 = void;
        using tp2 = void;
        static inline constexpr uint8_t fifoSize = 16;
    };
	struct Crsf3Config {
        using router = Devices::router;
		using bcastInterfaces = Meta::List<crsf2>;
        static inline constexpr uint8_t id = 2;
        using src = crsf_in::input;
        using dest = crsf_in::messageBuffer;
        using rxpin = crsf3_rx;
        using txpin = crsf3_tx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfFd4DmaChannelComponent1;
		using dmaChWrite = csrfFd4DmaChannelComponent2;
        using storage =  StorageAdapter<1>;
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

        led1::template dir<Mcu::Output>();
        led2::template dir<Mcu::Output>();

        crsf_in::init();
        crsf_in::baud(420'000);

        crsf2::init();
        crsf3::init();

#ifdef USE_DEBUG
        debug::init();
#else
#endif
    }
};

