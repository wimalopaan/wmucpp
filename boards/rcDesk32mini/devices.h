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
#include "rc/sbus2_2.h"
#include "rc/sbus_2.h"
#include "rc/sumdv3_2.h"
#include "rc/crsf_2.h"
#include "rc/hwext.h"
#include "rc/package_relay_rewrite.h"
#include "stdapp/stdcomp.h"
#include "blinker.h"
#include "debug_2.h"
#include "button.h"
#include "adcadapter.h"

struct WeAct;

using namespace std::literals::chrono_literals;

template<typename HW, typename Config, typename MCU = DefaultMcu> struct Devices;

template<typename Config, typename MCU>
struct Devices<WeAct, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using storage = Config::storage;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;
	using auxDmaChannel1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
	using auxDmaChannel2 = Mcu::Components::DmaChannel<typename dma1::component_t, 2>;
	using adcDmaChannel  = Mcu::Components::DmaChannel<typename dma1::component_t, 3>;

    using tx = Mcu::Stm::Pin<gpioa, 9, MCU>;
    using rx = Mcu::Stm::Pin<gpioa, 10, MCU>;

	using gfsm = Config::gfsm;
	
#if defined(USE_HWEXT)
	struct HwExtConfig;
    using hwext = External::EdgeTx::HwExtension<1, HwExtConfig, MCU>;
	using sbus = void;
	using modcom = void;
	using crsf = void;
#elif defined(USE_SBUS)
	using hwext = void;
	struct SBusConfig;
	using sbus = RC::Protokoll::SBus::V2::Output<1, SBusConfig, MCU>;
	
	struct ModComConfig;
	using modcom = External::EdgeTx::ModuleCom<1, ModComConfig, MCU>;
	using crsf = void;
#elif defined(USE_CRSF)
	using hwext = void;
	using sbus = void;
	using modcom = void;
	struct CrsfConfig;
    using crsf = RC::Protokoll::Crsf::V4::PacketRelayRewrite<1, CrsfConfig, MCU>;
#else
# warning "wrong protocol selection"
#endif

#ifdef SERIAL_DEBUG
    using debugtx = Mcu::Stm::Pin<gpioa, 2, MCU>; 
    struct DebugConfig;
    using debug = SerialBuffered<2, DebugConfig, MCU>;
#else
	using debug = void;
#endif
	
    using vin0 = Mcu::Stm::Pin<gpioa, 0, MCU>; // adc in0
	using vin1 = Mcu::Stm::Pin<gpioa, 1, MCU>; // adc in1
#ifndef SERIAL_DEBUG
	using vin2 = Mcu::Stm::Pin<gpioa, 2, MCU>; // adc in2 -> debugtx
#endif
	using vin3 = Mcu::Stm::Pin<gpioa, 3, MCU>; // adc in3
	// using vin3 = Mcu::Stm::Pin<gpioa, 4, MCU>; // adc in4 -> LED
	using vin4 = Mcu::Stm::Pin<gpioa, 5, MCU>; // adc in5
	using vin5 = Mcu::Stm::Pin<gpioa, 6, MCU>; // adc in6
	using vin6 = Mcu::Stm::Pin<gpioa, 7, MCU>; // adc in7
	using vin7 = Mcu::Stm::Pin<gpioa, 8, MCU>; // adc in8

#ifdef SERIAL_DEBUG
	using analogs = Meta::List<vin0, vin1, vin3, vin4, vin5, vin6, vin7>;
#else	
	using analogs = Meta::List<vin0, vin1, vin2, vin3, vin4, vin5, vin6, vin7>;
#endif
	struct AdcConfig;
    using adc = Mcu::Stm::V4::Adc<1, AdcConfig>;
	
	struct AdcAdapterConfig;
	using adcAdapter = AdcAdapter<AdcAdapterConfig>;
	
	// Led
    using led = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using invLed = Mcu::Stm::Gpio::Inverter<led>;
    using ledBlinker = External::Blinker<invLed, systemTimer>;

#ifdef USE_BUTTON
	using button = Mcu::Stm::Pin<gpioa, 14, MCU>;
    using buttonInv = Mcu::Stm::Gpio::Inverter<button, true>;
    using btn = External::Button<buttonInv, systemTimer, External::Tick<systemTimer>{300ms}.raw(),
                                 External::Tick<systemTimer>{3000ms}.raw(), void>;
#else
	using btn = void;
#endif
	
	using d0 = Mcu::Stm::Pin<gpiob, 6, MCU>;
	using d1 = Mcu::Stm::Pin<gpiob, 7, MCU>;
	using d2 = Mcu::Stm::Pin<gpioc, 14, MCU>;
	using d3 = Mcu::Stm::Pin<gpioc, 15, MCU>;
	
	using digitals = Meta::List<d0, d1, d2, d3>;
	
#ifdef SERIAL_DEBUG
	struct DebugConfig {
        using pin = debugtx;
        using clock = Devices::clock;
        static inline constexpr uint16_t bufferSize = 256;
    };
#endif
	struct HwExtConfig {
        using clock = Devices::clock;
        using systemTimer = Devices::systemTimer;
        using dmaChComponent = auxDmaChannel1;
        using pin = tx;
        using input = void;
        using storage = Devices::storage;
        using debug = Devices::debug;
        using tp = void;
    };
	struct SBusConfig {
        using clock = Devices::clock;
        using debug = Devices::debug;
        using dmaChComponent = auxDmaChannel1;
        using systemTimer = Devices::systemTimer;
		using adapter = void;
        using pin = tx;
        using tp = void;
    };
	struct AdcConfig {
        using debug = void;
#ifdef SERIAL_DEBUG
		using channels = std::integer_sequence<uint8_t, 0, 1, 3, 5, 6, 7, 8>;
#else
        using channels = std::integer_sequence<uint8_t, 0, 1, 2, 3, 5, 6, 7, 8>;
#endif
		static_assert(Meta::size_v<analogs> == channels::size());
        using dmaChannel = adcDmaChannel;
        using trigger = Mcu::Stm::ContinousSampling<2>;
        using isrConfig = Meta::List<>;
    };
	struct ModComConfig {
		using clock = Devices::clock;
        using debug = Devices::debug;
        using dmaChRead = auxDmaChannel1;
		using dmaChWrite = auxDmaChannel2;
        using systemTimer = Devices::systemTimer;
        using txpin = tx;
		using rxpin = rx;
        using tp = void;
		using callback = gfsm;
		using storage = Devices::storage;
	};
	struct CrsfConfig {
        using src = adcAdapter;
        using dest = void;
		using bcastInterfaces = Meta::List<>;
        using rxpin = rx;
        using txpin = tx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = auxDmaChannel1;
        using dmaChWrite = auxDmaChannel2;
        using storage = Devices::storage;
        using debug = Devices::debug;
        using tp = void;
        static inline constexpr uint8_t fifoSize = 16;
    };
	struct AdcAdapterConfig {
		using adc = Devices::adc;
		using storage = Devices::storage;
	};
	
	using periodics = StandardComponents<debug, sbus, modcom, crsf, ledBlinker, btn, adcAdapter>;
	
	static inline void periodic() {
        periodics::periodic();
    }
    static inline void ratePeriodic() {
        periodics::ratePeriodic();
    }
	static inline void init() {
        clock::init();

		SYSCFG->CFGR1 |= (SYSCFG_CFGR1_PA12_RMP | SYSCFG_CFGR1_PA11_RMP); // PA9 (tx), PA10 (rx)
		
		systemTimer::init();

        gpioa::init();
        gpiob::init();
        gpioc::init();

        dma1::init();

        led::template dir<Mcu::Output>();
        ledBlinker::event(ledBlinker::Event::Off);

#ifdef USE_BUTTON
        btn::template init<false>();
#endif
		
		Meta::visit<digitals>([]<typename DI>(Meta::Wrapper<DI>){
								  DI::template dir<Mcu::Input>();
								  DI::template pullup<true>();
							  });
		Meta::visit<analogs>([]<typename VI>(Meta::Wrapper<VI>){
								 VI::analog();
							 });
		
		adc::init();
		adc::start();

#ifdef SERIAL_DEBUG
		debug::init();
#endif
    }
};
