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
#include "dac.h"
#include "adc.h"
#include "opamp.h"
#include "tick.h"
#include "meta.h"
#include "rc/rc_2.h"
#include "rc/crsf_2.h"
#include "stdapp/stdcomp.h"
#include "blinker.h"
#include "debug_2.h"
#include "ams5048.h"
#include "focservo.h"

#include "crsf_cb.h"

struct Foc01;

using namespace std::literals::chrono_literals;

template<typename HW, typename Config, typename MCU = DefaultMcu>
struct Devices;

template<typename Config, typename MCU>
struct Devices<Foc01, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 8'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    //debug?
    using trace = Arm::V3::Trace<clock, 10_MHz, 1024>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    using storage = Config::storage;

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;
    using csrfInDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
    using spiDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 2>;
    using spiDmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma1::component_t, 3>;
	using adc1DmaChannel = Mcu::Stm::Dma::Channel<dma1, 4, MCU>;
	using adc2DmaChannel = Mcu::Stm::Dma::Channel<dma1, 5, MCU>;

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
    using ledPin = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using led = External::Blinker<ledPin, systemTimer>;

    // Testpin
    using tp = Mcu::Stm::Gpio::ActiveHigh<Mcu::Stm::Pin<gpioa, 8, MCU>>;

    // using mco = Mcu::Stm::Pin<gpioa, 8, MCU>;

	struct PwmConfig;
	using pwm = Mcu::Stm::V3::Pwm::ThreePhases<4, PwmConfig>;
	using pwm1 = Mcu::Stm::Pin<gpiob, 6, MCU>;
	using pwm2 = Mcu::Stm::Pin<gpiob, 7, MCU>;
	using pwm3 = Mcu::Stm::Pin<gpiob, 8, MCU>;
	
    // SPI 2:
    using spi_cs = Mcu::Stm::Pin<gpiof, 0, MCU>; // Tauschen mit ENA (PF0 -> PA12)
    using spi_miso = Mcu::Stm::Pin<gpioa, 10, MCU>;
    using spi_mosi = Mcu::Stm::Pin<gpioa, 11, MCU>;
    using spi_clk = Mcu::Stm::Pin<gpiof, 1, MCU>;

    using mps_fault = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using mps_enable = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using mps_sleep = Mcu::Stm::Pin<gpiob, 4, MCU>;

	struct AmsConfig;
	using ams5048 = External::AMS5048<2, AmsConfig>;

	struct ServoConfig;
	using servo = FocServo<ServoConfig>;
	
	using dac = Mcu::Stm::V3::Dac<1, MCU>;
	
    using adc1DmaStorage = std::array<volatile uint16_t, 4>;
	using adc2DmaStorage = std::array<volatile uint16_t, 4>;
    using adc1 = Mcu::Stm::V3::Adc<1, Meta::NList<13>, pwm, adc1DmaChannel, adc1DmaStorage, Meta::List<Mcu::Stm::EndOfSequence>, MCU>; // opamp1
	using adc2 = Mcu::Stm::V3::Adc<2, Meta::NList<16, 18>, pwm, adc2DmaChannel, adc2DmaStorage, Meta::List<Mcu::Stm::EndOfSequence>, MCU>; // opamp1

	using soa = Mcu::Stm::Pin<gpioa, 0, MCU>;
	using soa2 = Mcu::Stm::Pin<gpiob, 0, MCU>;
	using sob = Mcu::Stm::Pin<gpioa, 1, MCU>;
	using soc = Mcu::Stm::Pin<gpioa, 2, MCU>;
	// using isense = Mcu::Stm::Pin<gpioa, 3, MCU>;
	using soc2 = Mcu::Stm::Pin<gpioa, 3, MCU>;
	
	using follow1 = Mcu::Stm::V2::Follower<1, 1, MCU>;
	using follow2 = Mcu::Stm::V2::Follower<2, 2, MCU>;
	using follow3 = Mcu::Stm::V2::Follower<3, 2, MCU>;
	
    using components = StandardComponents<trace, debug, crsf, led, ams5048, servo>;

    struct CrsfCallbackConfig {
        using timer = systemTimer;
        using crsf = Devices::crsf;
        using storage = Devices::storage;
		using servo = Devices::servo;
    };
    struct CrsfConfig {
        using txpin = crsftx;
        using rxpin = crsftx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRW  = csrfInDmaChannelComponent1;
        using debug = void;
        using tp = void;
        using callback = CrsfCallback<CrsfCallbackConfig, Devices::debug>;
        static inline constexpr uint8_t fifoSize = 8;
    };
	struct PwmConfig {
		using debug = Devices::debug;
		using clock = Devices::clock;
		using systemTimer = Devices::systemTimer;
		using pin1 = pwm1;
		using pin2 = pwm2;
		using pin3 = pwm3;
	};
	struct AmsConfig {
        using systemTimer = Devices::systemTimer;
        using debug = void;
        // using debug = Devices::debug;
		using tp = void;
        using cs = spi_cs;
        using miso = spi_miso;
        using mosi = spi_mosi;
        using clk = spi_clk;
        using txDmaComponent = spiDmaChannelComponent1;
        using rxDmaComponent = spiDmaChannelComponent2;
    };
	struct AdcAdapter {
		template<uint8_t Ch>
		static inline uint16_t value() {
			if constexpr(Ch == 0) {
				return adc1::values()[0] - 2048;
			}
			else if constexpr (Ch == 1) {
				return adc2::values()[0] - 2048;
			}
			else if constexpr (Ch == 2) {
				return adc2::values()[1] - 2048;
			}
			else {
				static_assert(false);
			}
		}
	};
	struct ServoConfig {
		using debug = Devices::debug;
		using timer = systemTimer;
		using pwm = Devices::pwm;
		using sensor = Devices::ams5048;
		using input = Devices::crsf::input;
		using adc = AdcAdapter;
	};
	
    static inline void init() {
		StandardComponents<clock, systemTimer, gpioa, gpiob, gpioc, gpiof, 
				adc1, adc2, dma1, pwm, dac, follow1, follow2, follow3, tp>::init();
		components::init();
		
		led::event(led::Event::Steady);

		mps_sleep::template dir<Mcu::Output>();
		mps_sleep::set();
		mps_enable::template dir<Mcu::Output>();
		mps_enable::set();
		
		dac::set(2048);
		
		soa::analog();
		soa2::analog();
		sob::analog();
		soc::analog();
		soc2::analog();
		// isense::analog();
		adc1::start();
		adc2::start();
    }
    static inline void periodic() {
        components::periodic();
    }
    static inline void ratePeriodic() {
        components::ratePeriodic();
    }
};

