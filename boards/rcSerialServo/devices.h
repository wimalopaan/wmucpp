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

#include "etl/fixedvector.h"
#include "etl/stackstring.h"
#include "etl/algorithm.h"

#include "meta.h"
#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "mcu/eeprom.h"
#include "components.h"
#include "adc.h"
#include "usart.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "clock.h"
#include "gpio.h"
#include "tick.h"
#include "meta.h"
#include "rc/rc_2.h"
#include "rc/rc_2.h"
#include "rc/crsf_2.h"
#include "rc/waveshare_2.h"
#include "stdapp/stdcomp.h"
#include "blinker.h"
#include "debug_2.h"
#include "button.h"
#include "watchdog.h"

#include "crsf_cb.h"
#include "servo_cb.h"

struct WeAct;
struct Wmg0b1;
struct Nucleo;

using namespace std::literals::chrono_literals;

template<typename HW, typename Config, typename MCU = DefaultMcu> struct Devices;

template<typename Config, typename MCU>
struct Devices<WeAct, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    struct WdgConfig {
        static inline constexpr uint32_t reload = 500; // 500ms
    };
    using watchDog = WatchDog<WdgConfig>;
    
    using storage = Config::storage;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;
	using srvDmaChannel1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
	// using srvDmaChannel2 = Mcu::Components::DmaChannel<typename dma1::component_t, 2>;
    using crsfDmaChannel1 = Mcu::Components::DmaChannel<typename dma1::component_t, 3>;
    using crsfDmaChannel2 = Mcu::Components::DmaChannel<typename dma1::component_t, 4>;
    // using adcDmaChannel  = Mcu::Components::DmaChannel<typename dma1::component_t, 5>;

    // UART1
    using servotx = Mcu::Stm::Pin<gpioa, 9, MCU>;
    using servorx = Mcu::Stm::Pin<gpioa, 10, MCU>;

    struct WSConfig;
    using srv_waveshare = External::WaveShare::V4::Servo<1, WSConfig, MCU>;
    
    // UART2
    using inputtx = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using inputrx = Mcu::Stm::Pin<gpioa, 3, MCU>;

    struct CrsfConfig;
    using crsf = RC::Protokoll::Crsf::V4::Master<2, CrsfConfig, MCU>;
    using crsf_in = crsf::input;

	using debug = void;
	
	// Led
    using led = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using invLed = Mcu::Stm::Gpio::Inverter<led>;
    using ledBlinker = External::Blinker<invLed, systemTimer>;

	using btn = void;
	
    struct CrsfCallbackConfig;
    using crsf_cb = CrsfCallback<CrsfCallbackConfig, debug>;
    
    struct ServoCallbackConfig;
    using servo_cb = ServoCallback<ServoCallbackConfig>;
    
    struct CrsfConfig {
        using rxpin = inputrx;
        using txpin = inputtx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = crsfDmaChannel1;
        using dmaChWrite = crsfDmaChannel2;
        using debug = Devices::debug;
        using tp = void;
        using callback = crsf_cb;
        static inline constexpr uint8_t fifoSize = 16;
    };
    struct CrsfCallbackConfig {
        using src = Devices::crsf;
        using srv = srv_waveshare;
        using storage = Config::storage;
        using watchdog = Devices::watchDog;
        using timer = systemTimer;
        using tp = void;
        using debug = Devices::debug;
    };
    struct WSConfig {
        using pin = servotx;
        using polar = void;
        using dmaChComponent = srvDmaChannel1;
        using timer = systemTimer;
        using clk = clock;
        using callback = servo_cb;
        using tp = void;
        using dbg = debug;
        using storage = Devices::storage;
    };
    struct ServoCallbackConfig {
        using systemTimer = Devices::systemTimer;
        using srv = Devices::srv_waveshare;
        using crsf_cb = Devices::crsf_cb;
        using debug = Devices::debug;
    };
    using periodics = StandardComponents<debug, watchDog, crsf, srv_waveshare, ledBlinker, btn>;
	
	static inline void periodic() {
        periodics::periodic();
    }
    static inline void ratePeriodic() {
        periodics::ratePeriodic();
    }
	static inline void init() {
        clock::init();

#ifdef SYSCFG_CFGR1_PA12_RMP
		SYSCFG->CFGR1 |= (SYSCFG_CFGR1_PA12_RMP | SYSCFG_CFGR1_PA11_RMP); // PA9 (tx), PA10 (rx)
#endif
		
		systemTimer::init();

        gpioa::init();
        gpiob::init();
        gpioc::init();

        dma1::init();

        ledBlinker::init();
        ledBlinker::event(ledBlinker::Event::Off);

        crsf::init();
        crsf::baud(RC::Protokoll::Crsf::V4::baudrate);
        
        srv_waveshare::init();
        
        watchDog::init();
        
    }
};

template<typename Config, typename MCU>
struct Devices<Wmg0b1, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    struct WdgConfig {
        static inline constexpr uint32_t reload = 500; // 500ms
    };
    using watchDog = WatchDog<WdgConfig>;

    using storage = Config::storage;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;
    using srvDmaChannel1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
    // using srvDmaChannel2 = Mcu::Components::DmaChannel<typename dma1::component_t, 2>;
    using crsfDmaChannel1 = Mcu::Components::DmaChannel<typename dma1::component_t, 3>;
    using crsfDmaChannel2 = Mcu::Components::DmaChannel<typename dma1::component_t, 4>;
    // using adcDmaChannel  = Mcu::Components::DmaChannel<typename dma1::component_t, 5>;

    using gfsm = Config::gfsm;

#ifdef ALTERNATE_PINS
    // UART6
    using servotx = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using servorx = Mcu::Stm::Pin<gpiob, 9, MCU>;
#else
    // UART2
    using servotx = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using servorx = Mcu::Stm::Pin<gpioa, 3, MCU>;

    struct WSConfig;
    using srv_waveshare = External::WaveShare::V4::Servo<2, WSConfig, MCU>;
#endif

    // UART1
    using inputtx = Mcu::Stm::Pin<gpioa, 9, MCU>;
    using inputrx = Mcu::Stm::Pin<gpioa, 10, MCU>;

#ifdef USE_TP
    using tp = Mcu::Stm::Pin<gpiob, 4, MCU>;
    // using tp1 = Mcu::Stm::Pin<gpiob, 6, MCU>;
    // using tp2 = Mcu::Stm::Pin<gpiob, 7, MCU>;
#else 
    using tp = void;
#endif

    struct CrsfConfig;
    using crsf = RC::Protokoll::Crsf::V4::Master<1, CrsfConfig, MCU>;
    using crsf_in = crsf::input;

#ifdef SERIAL_DEBUG
    // UART5
    using debugtx = Mcu::Stm::Pin<gpiob, 3, MCU>;
    struct DebugConfig;
    using debug = SerialBuffered<5, DebugConfig, MCU>;
#else
    using debug = void;
#endif

    // Led
    using led = Mcu::Stm::Pin<gpiob, 2, MCU>;
    using ledBlinker = External::Blinker<led, systemTimer>;
    using led2 = Mcu::Stm::Pin<gpioc, 6, MCU>;
    using ledBlinker2 = External::Blinker<led2, systemTimer>;

#ifdef SERIAL_DEBUG
    struct DebugConfig {
        using pin = debugtx;
        using clock = Devices::clock;
        static inline constexpr uint16_t bufferSize = 128;
    };
#endif
    struct CrsfCallbackConfig;
    using crsf_cb = CrsfCallback<CrsfCallbackConfig, debug>;
    
    struct ServoCallbackConfig;
    using servo_cb = ServoCallback<ServoCallbackConfig>;
    
    struct CrsfConfig {
        using rxpin = inputrx;
        using txpin = inputtx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = crsfDmaChannel1;
        using dmaChWrite = crsfDmaChannel2;
        using debug = Devices::debug;
        using tp = void;
        using callback = crsf_cb;
        static inline constexpr uint8_t fifoSize = 16;
    };
    struct CrsfCallbackConfig {
        using src = Devices::crsf;
        using srv = srv_waveshare;
        using storage = Config::storage;
        using watchdog = Devices::watchDog;
        using timer = systemTimer;
        using tp = void;
        using debug = Devices::debug;
    };
    struct WSConfig {
        using pin = servotx;
        using polar = void;
        using dmaChComponent = srvDmaChannel1;
        using timer = systemTimer;
        using clk = clock;
        using callback = servo_cb;
        using tp = Devices::tp;
        using dbg = debug;
        using storage = Devices::storage;
    };
    struct ServoCallbackConfig {
        using systemTimer = Devices::systemTimer;
        using srv = Devices::srv_waveshare;
        using crsf_cb = Devices::crsf_cb;
        using debug = Devices::debug;
    };

    using periodics = StandardComponents<debug, watchDog, crsf, srv_waveshare, servo_cb, ledBlinker, ledBlinker2>;

    static inline void periodic() {
        periodics::periodic();
    }
    static inline void ratePeriodic() {
        periodics::ratePeriodic();
    }
    static inline void init() {
        clock::init();
        systemTimer::init();

        gpioa::init();
        gpiob::init();
        gpioc::init();

        dma1::init();

        ledBlinker::init();
        ledBlinker::event(ledBlinker::Event::Steady);

        ledBlinker2::init();
        ledBlinker2::event(ledBlinker2::Event::Steady);

#ifdef SERIAL_DEBUG
        debug::init();
#endif
        crsf::init();
        crsf::baud(RC::Protokoll::Crsf::V4::baudrate);

        srv_waveshare::init();
        
        watchDog::init();
        
#ifdef USE_TP
        tp::template dir<Mcu::Output>();
        // tp1::template dir<Mcu::Output>();
        // tp2::template dir<Mcu::Output>();
#endif
    }
};
