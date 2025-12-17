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
#include "pwm.h"
#include "adc.h"
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
#include "telemetry.h"

#include "crsf_cb.h"
#include "switch_cb.h"
#include "pattern.h"
#include "slaveswitch.h"

struct Nucleo;
struct WeAct;
struct SW20; // board: RCMultiSwitchSmall10
struct SW21; // board: RCMultiSwitchSmall11 (one sided version, voltage sensor)
struct SW22; // board: RCMultiSwitchSmall12 (EasyEda, OSHWLAB, one sided version, voltage sensor, Input 0, 1)

using namespace std::literals::chrono_literals;

template<typename HW, typename Config, typename MCU = DefaultMcu> struct Devices;

template<typename Config, typename MCU>
struct Devices<SW22, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using storage = Config::storage;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpioa, 9, MCU>;
    using crsfrx = Mcu::Stm::Pin<gpioa, 10, MCU>;

    // Usart 1: CRSF
    using csrfInDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
    using csrfInDmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma1::component_t, 2>;
    struct CrsfConfig;
    using crsf = RC::Protokoll::Crsf::V4::Master<1, CrsfConfig, MCU>;
    using crsfBuffer = crsf::messageBuffer;

#ifdef SERIAL_DEBUG
    using debugtx = Mcu::Stm::Pin<gpioa, 15, MCU>;
    struct DebugConfig;
    using debug = SerialBuffered<2, DebugConfig, MCU>;
    struct DebugConfig {
        using pin = debugtx;
        using clock = Devices::clock;
        static inline constexpr bool rxtxswap = true;
        static inline constexpr uint16_t bufferSize = 256;
    };
#else
    using debug = void;
#endif

    using adcDmaChannel = Mcu::Components::DmaChannel<typename dma1::component_t, 3>;

    using vin = Mcu::Stm::Pin<gpioa, 5, MCU>; // adc in5
    // ADC
    struct AdcConfig {
        using debug = void;
        using channels = std::integer_sequence<uint8_t, 5, 12>; // temp channel 12
        using dmaChannel = adcDmaChannel;
        using trigger = Mcu::Stm::ContinousSampling<256>;
        using isrConfig = Meta::List<>;
        static inline constexpr uint8_t slowChannel = 12; // temp
        static inline constexpr uint8_t slowSampleTime = 7;
    };
    using adc = Mcu::Stm::V4::Adc<1, AdcConfig>;

    static inline uint32_t r1 = 1'000;
    static inline uint32_t r2 = 10'000;
    static inline uint32_t Vref_n = 33;
    static inline uint32_t Vref_d = 10;
    static inline uint32_t Vcal_n = 30;

    struct TelemConfig {
        using debug = Devices::debug;
        using timer = systemTimer;
        using clock = Devices::clock;
        using messagebuffer = crsfBuffer;
        using storage = Devices::storage;
    };
    using telemetry = Telemetry<TelemConfig>;

    // Led
    using led = Mcu::Stm::Pin<gpiob, 2, MCU>;
    using ledBlinker = External::Blinker<led, systemTimer>;

    using btn = void;
    using in0 = Mcu::Stm::Pin<gpiob, 9, MCU>;
	using in1 = Mcu::Stm::Pin<gpioc, 14, MCU>;

    using tp1 = void;

    using sw0 = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using sw1 = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using sw2 = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using sw3 = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using sw4 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using sw5 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using sw6 = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using sw7 = Mcu::Stm::Pin<gpiob, 1, MCU>;

    struct SlaveSwitchProtocolConfig {
        using debug = Devices::debug;
        using timer = systemTimer;
        using clock = Devices::clock;
#ifdef USE_SLAVE_COMMAND
        using messagebuffer = crsfBuffer;
#else
        using messagebuffer = void;
#endif
        using storage = Devices::storage;
    };
    using ssp = SlaveSwitchProtocol<SlaveSwitchProtocolConfig>;

    template<auto N, typename Pin, typename Callback>
    struct SlaveProtocolAdapter {
        static inline constexpr auto number = Pin::number;
        using component_t = Pin::component_t;
        static inline constexpr void afunction(const uint8_t af) {
            Pin::afunction(af);
        }
        static inline constexpr void set() {
            Pin::set();
            Callback::set(N);
        }
        static inline constexpr void reset() {
            Pin::reset();
            Callback::reset(N);
        }
        template<typename D>
        static inline void dir() {
            Pin::template dir<D>();
        }
    };
    using spa0 = SlaveProtocolAdapter<0, sw0, ssp>;
    using spa1 = SlaveProtocolAdapter<1, sw1, ssp>;
    using spa2 = SlaveProtocolAdapter<2, sw2, ssp>;
    using spa3 = SlaveProtocolAdapter<3, sw3, ssp>;
    using spa4 = SlaveProtocolAdapter<4, sw4, ssp>;
    using spa5 = SlaveProtocolAdapter<5, sw5, ssp>;
    using spa6 = SlaveProtocolAdapter<6, sw6, ssp>;
    using spa7 = SlaveProtocolAdapter<7, sw7, ssp>;

    struct MasterPwmConfig;
    struct SlavePwmConfig;
	
    using pwm2 = Mcu::Stm::V3::Pwm::Simple<2, MasterPwmConfig>;
    using pwm3 = Mcu::Stm::V3::Pwm::Simple<3, SlavePwmConfig>;
    struct MasterPwmConfig {
        using clock = Devices::clock;
        static inline constexpr auto syncMode = Mcu::Stm::Timers::SyncMode::Master;
    };
    struct SlavePwmConfig {
        using clock = Devices::clock;
        static inline constexpr auto syncMode = Mcu::Stm::Timers::SyncMode::Slave;
        using master = pwm2;
    };

    // s0 : pa0  : tim2 ch1
    // s1 : pa1  : tim2 ch2
    // s2 : pa2  : tim2 ch3
    // s3 : pa3  : tim2 ch4
    // s4 : pa6  : tim3 ch1
    // s5 : pa7  : tim3 ch2
    // s6 : pb0  : tim3 ch3
    // s7 : pb1  : tim3 ch4

    using debug1 = void;
    using adap0 = Local::PwmAdapter<pwm2, 1, false, true, debug1>;
    using adap1 = Local::PwmAdapter<pwm2, 2, false, true, debug1>;
    using adap2 = Local::PwmAdapter<pwm2, 3, false, true, debug1>;
    using adap3 = Local::PwmAdapter<pwm2, 4, false, true, debug1>;
    using adap4 = Local::PwmAdapter<pwm3, 1, false, true, debug1>;
    using adap5 = Local::PwmAdapter<pwm3, 2, false, true, debug1>;
    using adap6 = Local::PwmAdapter<pwm3, 3, false, true, debug1>;
    using adap7 = Local::PwmAdapter<pwm3, 4, false, true, debug1>;

#ifdef USE_MORSE
    struct BConfig {
        using timer = systemTimer;
        using debug = debug1;
        static inline constexpr auto& text = storage::eeprom.morse_text;
    };
    using bsw0 = External::Morse::BlinkerWithPwm<spa0, BConfig, adap0>;
    using bsw1 = External::Morse::BlinkerWithPwm<spa1, BConfig, adap1>;
    using bsw2 = External::Morse::BlinkerWithPwm<spa2, BConfig, adap2>;
    using bsw3 = External::Morse::BlinkerWithPwm<spa3, BConfig, adap3>;
    using bsw4 = External::Morse::BlinkerWithPwm<spa4, BConfig, adap4>;
    using bsw5 = External::Morse::BlinkerWithPwm<spa5, BConfig, adap5>;
    using bsw6 = External::Morse::BlinkerWithPwm<spa6, BConfig, adap6>;
    using bsw7 = External::Morse::BlinkerWithPwm<spa7, BConfig, adap7>;
#else
    using bsw0 = External::BlinkerWithPwm<spa0, systemTimer, adap0, debug1>;
    using bsw1 = External::BlinkerWithPwm<spa1, systemTimer, adap1, debug1>;
    using bsw2 = External::BlinkerWithPwm<spa2, systemTimer, adap2, debug1>;
    using bsw3 = External::BlinkerWithPwm<spa3, systemTimer, adap3, debug1>;
    using bsw4 = External::BlinkerWithPwm<spa4, systemTimer, adap4, debug1>;
    using bsw5 = External::BlinkerWithPwm<spa5, systemTimer, adap5, debug1>;
    using bsw6 = External::BlinkerWithPwm<spa6, systemTimer, adap6, debug1>;
    using bsw7 = External::BlinkerWithPwm<spa7, systemTimer, adap7, debug1>;
#endif

    using bsws = Meta::List<bsw0, bsw1, bsw2, bsw3, bsw4, bsw5, bsw6, bsw7>;

	struct PatGenConfig;
	using patgen0 = External::Pattern::Generator<0, PatGenConfig>;
	using patgen1 = External::Pattern::Generator<1, PatGenConfig>;
	using patgen2 = External::Pattern::Generator<2, PatGenConfig>;
	using patgen3 = External::Pattern::Generator<3, PatGenConfig>;
	
	struct PatGenConfig {
		using timer = systemTimer;
		using storage = Devices::storage;
        using debug = Devices::debug;
		using outputs = bsws;
		using messagebuffer = crsfBuffer;
	};
	
    struct SwitchCallbackConfig {
        using debug = Devices::debug;
        using storage = Devices::storage;
        using bsws = Devices::bsws;
		using patgen0 = Devices::patgen0;
		using patgen1 = Devices::patgen1;
		using patgen2 = Devices::patgen2;
		using patgen3 = Devices::patgen3;
    };

    struct CrsfCallbackConfig {
        using storage = Devices::storage;
        using debug = Devices::debug;
        using bswList = bsws;
        using pwmList = Meta::List<pwm3, pwm2>;
        using timer = systemTimer;
        using crsf = Devices::crsf;
        using telemetry = Devices::telemetry;
        using switchCallback = SwitchCallback<SwitchCallbackConfig>;
		using patgen0 = Devices::patgen0;
		using patgen1 = Devices::patgen1;
		using patgen2 = Devices::patgen2;
		using patgen3 = Devices::patgen3;
        using slave = Devices::ssp;
    };
    struct CrsfConfig {
        using txpin = crsftx;
#ifdef CRSF_HALFDUPLEX
        using rxpin = txpin;
#else
        using rxpin = crsfrx; // full-duplex
#endif
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfInDmaChannelComponent1;
        using dmaChWrite  = csrfInDmaChannelComponent2;
        using dmaChRW  = csrfInDmaChannelComponent1;
        using debug = void;
        // using debug = Devices::debug;
        using tp = tp1;
        using callback = CrsfCallback<CrsfCallbackConfig>;
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
#else
        in0::template dir<Mcu::Input>();
        in0::template pullup<true>();
		in1::template dir<Mcu::Input>();
        in1::template pullup<true>();
#endif
#ifdef USE_TP1
        tp1::template dir<Mcu::Output>();
#endif
        crsf::init();
#ifdef CRSF_TX_OPENDRAIN
        crsftx::template openDrain<true>();
        crsftx::template pullUp<true>();
#endif

        sw0::template dir<Mcu::Output>();
        sw1::template dir<Mcu::Output>();
        sw2::template dir<Mcu::Output>();
        sw3::template dir<Mcu::Output>();
        sw4::template dir<Mcu::Output>();
        sw5::template dir<Mcu::Output>();
        sw6::template dir<Mcu::Output>();
        sw7::template dir<Mcu::Output>();

        pwm2::init();
        pwm3::init();
        pwm2::start(); //synchron with pwm3;

        adc::init();
        adc::oversample(7);
    }
};

template<typename Config, typename MCU>
struct Devices<SW21, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using storage = Config::storage;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using crsfrx = Mcu::Stm::Pin<gpiob, 7, MCU>;

    // Usart 1: CRSF
    using csrfInDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
    using csrfInDmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma1::component_t, 2>;
    struct CrsfConfig;
    using crsf = RC::Protokoll::Crsf::V4::Master<1, CrsfConfig, MCU>;
    using crsfBuffer = crsf::messageBuffer;

#ifdef SERIAL_DEBUG
    using debugtx = Mcu::Stm::Pin<gpioa, 15, MCU>;
    struct DebugConfig;
    using debug = SerialBuffered<2, DebugConfig, MCU>;
    struct DebugConfig {
        using pin = debugtx;
        using clock = Devices::clock;
        static inline constexpr bool rxtxswap = true;
        static inline constexpr uint16_t bufferSize = 256;
    };
#else
    using debug = void;
#endif

    using adcDmaChannel = Mcu::Components::DmaChannel<typename dma1::component_t, 3>;

    using vin = Mcu::Stm::Pin<gpioa, 5, MCU>; // adc in5
    // ADC
    struct AdcConfig {
        using debug = void;
        using channels = std::integer_sequence<uint8_t, 5, 12>; // temp channel 12
        using dmaChannel = adcDmaChannel;
        using trigger = Mcu::Stm::ContinousSampling<256>;
        using isrConfig = Meta::List<>;
        static inline constexpr uint8_t slowChannel = 12; // temp
        static inline constexpr uint8_t slowSampleTime = 7;
    };
    using adc = Mcu::Stm::V4::Adc<1, AdcConfig>;

    static inline uint32_t r1 = 1'000;
    static inline uint32_t r2 = 10'000;
    static inline uint32_t Vref_n = 33;
    static inline uint32_t Vref_d = 10;
    static inline uint32_t Vcal_n = 30;

    struct TelemConfig {
        using debug = Devices::debug;
        using timer = systemTimer;
        using clock = Devices::clock;
        using messagebuffer = crsfBuffer;
        using storage = Devices::storage;
    };
    using telemetry = Telemetry<TelemConfig>;

    // Led
    using led = Mcu::Stm::Pin<gpioc, 15, MCU>;
    using ledBlinker = External::Blinker<led, systemTimer>;

    // Taster
#ifdef USE_BUTTON
    using button = Mcu::Stm::Pin<gpioa, 11, MCU>;
    using btn = External::Button<button, systemTimer, External::Tick<systemTimer>{300ms}.raw(),
                                 External::Tick<systemTimer>{3000ms}.raw(), void>;
    using in0 = void;
    using in1 = void;
#else
    using btn = void;
    using in0 = Mcu::Stm::Pin<gpioa, 11, MCU>;
    using in1 = void;
#endif

#ifdef USE_TP1
    using tp1 = Mcu::Stm::Pin<gpiob, 8, MCU>;
#else
    using tp1 = void;
#endif

    using sw0 = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using sw1 = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using sw2 = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using sw3 = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using sw4 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using sw5 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using sw6 = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using sw7 = Mcu::Stm::Pin<gpiob, 1, MCU>;

    struct SlaveSwitchProtocolConfig {
        using debug = Devices::debug;
        using timer = systemTimer;
        using clock = Devices::clock;
#ifdef USE_SLAVE_COMMAND
        using messagebuffer = crsfBuffer;
#else
        using messagebuffer = void;
#endif
        using storage = Devices::storage;
    };
    using ssp = SlaveSwitchProtocol<SlaveSwitchProtocolConfig>;

    template<auto N, typename Pin, typename Callback>
    struct SlaveProtocolAdapter {
        static inline constexpr auto number = Pin::number;
        using component_t = Pin::component_t;
        static inline constexpr void afunction(const uint8_t af) {
            Pin::afunction(af);
        }
        static inline constexpr void set() {
            Pin::set();
            Callback::set(N);
        }
        static inline constexpr void reset() {
            Pin::reset();
            Callback::reset(N);
        }
        template<typename D>
        static inline void dir() {
            Pin::template dir<D>();
        }
    };
    using spa0 = SlaveProtocolAdapter<0, sw0, ssp>;
    using spa1 = SlaveProtocolAdapter<1, sw1, ssp>;
    using spa2 = SlaveProtocolAdapter<2, sw2, ssp>;
    using spa3 = SlaveProtocolAdapter<3, sw3, ssp>;
    using spa4 = SlaveProtocolAdapter<4, sw4, ssp>;
    using spa5 = SlaveProtocolAdapter<5, sw5, ssp>;
    using spa6 = SlaveProtocolAdapter<6, sw6, ssp>;
    using spa7 = SlaveProtocolAdapter<7, sw7, ssp>;

    struct MasterPwmConfig;
    struct SlavePwmConfig;
	
    using pwm2 = Mcu::Stm::V3::Pwm::Simple<2, MasterPwmConfig>;
    using pwm3 = Mcu::Stm::V3::Pwm::Simple<3, SlavePwmConfig>;
    struct MasterPwmConfig {
        using clock = Devices::clock;
        static inline constexpr auto syncMode = Mcu::Stm::Timers::SyncMode::Master;
    };
    struct SlavePwmConfig {
        using clock = Devices::clock;
        static inline constexpr auto syncMode = Mcu::Stm::Timers::SyncMode::Slave;
        using master = pwm2;
    };

    // s0 : pa0  : tim2 ch1
    // s1 : pa1  : tim2 ch2
    // s2 : pa2  : tim2 ch3
    // s3 : pa3  : tim2 ch4
    // s4 : pa6  : tim3 ch1
    // s5 : pa7  : tim3 ch2
    // s6 : pb0  : tim3 ch3
    // s7 : pb1  : tim3 ch4

    using debug1 = void;
    using adap0 = Local::PwmAdapter<pwm2, 1, false, true, debug1>;
    using adap1 = Local::PwmAdapter<pwm2, 2, false, true, debug1>;
    using adap2 = Local::PwmAdapter<pwm2, 3, false, true, debug1>;
    using adap3 = Local::PwmAdapter<pwm2, 4, false, true, debug1>;
    using adap4 = Local::PwmAdapter<pwm3, 1, false, true, debug1>;
    using adap5 = Local::PwmAdapter<pwm3, 2, false, true, debug1>;
    using adap6 = Local::PwmAdapter<pwm3, 3, false, true, debug1>;
    using adap7 = Local::PwmAdapter<pwm3, 4, false, true, debug1>;

#ifdef USE_MORSE
    struct BConfig {
        using timer = systemTimer;
        using debug = debug1;
        static inline constexpr auto& text = storage::eeprom.morse_text;
    };
    using bsw0 = External::Morse::BlinkerWithPwm<spa0, BConfig, adap0>;
    using bsw1 = External::Morse::BlinkerWithPwm<spa1, BConfig, adap1>;
    using bsw2 = External::Morse::BlinkerWithPwm<spa2, BConfig, adap2>;
    using bsw3 = External::Morse::BlinkerWithPwm<spa3, BConfig, adap3>;
    using bsw4 = External::Morse::BlinkerWithPwm<spa4, BConfig, adap4>;
    using bsw5 = External::Morse::BlinkerWithPwm<spa5, BConfig, adap5>;
    using bsw6 = External::Morse::BlinkerWithPwm<spa6, BConfig, adap6>;
    using bsw7 = External::Morse::BlinkerWithPwm<spa7, BConfig, adap7>;
#else
    using bsw0 = External::BlinkerWithPwm<spa0, systemTimer, adap0, debug1>;
    using bsw1 = External::BlinkerWithPwm<spa1, systemTimer, adap1, debug1>;
    using bsw2 = External::BlinkerWithPwm<spa2, systemTimer, adap2, debug1>;
    using bsw3 = External::BlinkerWithPwm<spa3, systemTimer, adap3, debug1>;
    using bsw4 = External::BlinkerWithPwm<spa4, systemTimer, adap4, debug1>;
    using bsw5 = External::BlinkerWithPwm<spa5, systemTimer, adap5, debug1>;
    using bsw6 = External::BlinkerWithPwm<spa6, systemTimer, adap6, debug1>;
    using bsw7 = External::BlinkerWithPwm<spa7, systemTimer, adap7, debug1>;
#endif

    using bsws = Meta::List<bsw0, bsw1, bsw2, bsw3, bsw4, bsw5, bsw6, bsw7>;

	struct PatGenConfig;
	using patgen0 = External::Pattern::Generator<0, PatGenConfig>;
	using patgen1 = void;
	using patgen2 = void;
	using patgen3 = void;
	
	struct PatGenConfig {
		using timer = systemTimer;
		using storage = Devices::storage;
        using debug = Devices::debug;
		using outputs = bsws;
		using messagebuffer = crsfBuffer;
	};
	
    struct SwitchCallbackConfig;

    struct SwitchCallbackConfig {
        using debug = Devices::debug;
        using storage = Devices::storage;
        using bsws = Devices::bsws;
		using patgen0 = Devices::patgen0;
		using patgen1 = Devices::patgen1;
		using patgen2 = Devices::patgen2;
		using patgen3 = Devices::patgen3;
    };

    struct CrsfCallbackConfig {
        using storage = Devices::storage;
        using debug = Devices::debug;
        using bswList = bsws;
        using pwmList = Meta::List<pwm3, pwm2>;
        using timer = systemTimer;
        using crsf = Devices::crsf;
        using telemetry = Devices::telemetry;
        using switchCallback = SwitchCallback<SwitchCallbackConfig>;
		using patgen0 = Devices::patgen0;
		using patgen1 = Devices::patgen1;
		using patgen2 = Devices::patgen2;
		using patgen3 = Devices::patgen3;
        using slave = Devices::ssp;
    };
    struct CrsfConfig {
        using txpin = crsftx;
#ifdef CRSF_HALFDUPLEX
        using rxpin = txpin;
#else
        using rxpin = crsfrx; // full-duplex
#endif
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfInDmaChannelComponent1;
        using dmaChWrite  = csrfInDmaChannelComponent2;
        using dmaChRW  = csrfInDmaChannelComponent1;
        using debug = void;
        // using debug = Devices::debug;
        using tp = tp1;
        using callback = CrsfCallback<CrsfCallbackConfig>;
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
#else
        in0::template dir<Mcu::Input>();
        in0::template pullup<true>();
#endif
#ifdef USE_TP1
        tp1::template dir<Mcu::Output>();
#endif
        crsf::init();
#ifdef CRSF_TX_OPENDRAIN
        crsftx::template openDrain<true>();
        crsftx::template pullUp<true>();
#endif

        sw0::template dir<Mcu::Output>();
        sw1::template dir<Mcu::Output>();
        sw2::template dir<Mcu::Output>();
        sw3::template dir<Mcu::Output>();
        sw4::template dir<Mcu::Output>();
        sw5::template dir<Mcu::Output>();
        sw6::template dir<Mcu::Output>();
        sw7::template dir<Mcu::Output>();

        pwm2::init();
        pwm3::init();
        pwm2::start(); //synchron with pwm3;

        adc::init();
        adc::oversample(7);
    }
};

template<typename Config, typename MCU>
struct Devices<SW20, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using storage = Config::storage;

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
    using crsfBuffer = crsf::messageBuffer;

#ifdef SERIAL_DEBUG
    using debugtx = Mcu::Stm::Pin<gpiob, 6, MCU>;
    struct DebugConfig;
    using debug = SerialBuffered<1, DebugConfig, MCU>;
    struct DebugConfig {
        using pin = debugtx;
        using clock = Devices::clock;
        static inline constexpr bool rxtxswap = false;
        static inline constexpr uint16_t bufferSize = 64;
    };
#else
    using debug = void;
#endif

    using adc = void;

    using telemetry = void;

    // Led
    using led = Mcu::Stm::Pin<gpioc, 15, MCU>;
    using ledBlinker = External::Blinker<led, systemTimer>;

    // Taster
#ifdef USE_BUTTON
    using button = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using btn = External::Button<button, systemTimer, External::Tick<systemTimer>{300ms}.raw(),
                                 External::Tick<systemTimer>{3000ms}.raw(), void>;
#else
    using btn = void;
#endif
    using in0 = void;
    using in1 = void;
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

    struct SlaveSwitchProtocolConfig {
        using debug = Devices::debug;
        using timer = systemTimer;
        using clock = Devices::clock;
#ifdef USE_SLAVE_COMMAND
        using messagebuffer = crsfBuffer;
#else
        using messagebuffer = void;
#endif
        using storage = Devices::storage;
    };
    using ssp = SlaveSwitchProtocol<SlaveSwitchProtocolConfig>;

    template<auto N, typename Pin, typename Callback>
    struct SlaveProtocolAdapter {
        static inline constexpr auto number = Pin::number;
        using component_t = Pin::component_t;
        static inline constexpr void afunction(const uint8_t af) {
            Pin::afunction(af);
        }
        static inline constexpr void set() {
            Pin::set();
            Callback::set(N);
        }
        static inline constexpr void reset() {
            Pin::reset();
            Callback::reset(N);
        }
        template<typename D>
        static inline void dir() {
            Pin::template dir<D>();
        }
    };
    using spa0 = SlaveProtocolAdapter<0, sw0, ssp>;
    using spa1 = SlaveProtocolAdapter<1, sw1, ssp>;
    using spa2 = SlaveProtocolAdapter<2, sw2, ssp>;
    using spa3 = SlaveProtocolAdapter<3, sw3, ssp>;
    using spa4 = SlaveProtocolAdapter<4, sw4, ssp>;
    using spa5 = SlaveProtocolAdapter<5, sw5, ssp>;
    using spa6 = SlaveProtocolAdapter<6, sw6, ssp>;
    using spa7 = SlaveProtocolAdapter<7, sw7, ssp>;

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

#ifdef USE_MORSE
    struct BConfig {
        using timer = systemTimer;
        using debug = void;
        static inline constexpr auto& text = storage::eeprom.morse_text;
    };
    struct BConfigDebug {
        using timer = systemTimer;
        using debug = Devices::debug;
        static inline constexpr auto& text = storage::eeprom.morse_text;
    };
    using bsw0 = External::Morse::BlinkerWithPwm<spa0, BConfig, adap0>;
    using bsw1 = External::Morse::BlinkerWithPwm<spa1, BConfig, adap1>;
    using bsw2 = External::Morse::BlinkerWithPwm<spa2, BConfig, adap2>;
    using bsw3 = External::Morse::BlinkerWithPwm<spa3, BConfig, adap3>;
    using bsw4 = External::Morse::BlinkerWithPwm<spa4, BConfig, adap4>;
    using bsw5 = External::Morse::BlinkerWithPwm<spa5, BConfig, adap5>;
    using bsw6 = External::Morse::BlinkerWithPwm<spa6, BConfig, adap6>;
    using bsw7 = External::Morse::BlinkerWithPwm<spa7, BConfig, adap7>;
#else
    using bsw0 = External::BlinkerWithPwm<spa0, systemTimer, adap0, debug1>;
    using bsw1 = External::BlinkerWithPwm<spa1, systemTimer, adap1, debug1>;
    using bsw2 = External::BlinkerWithPwm<spa2, systemTimer, adap2, debug1>;
    using bsw3 = External::BlinkerWithPwm<spa3, systemTimer, adap3, debug1>;
    using bsw4 = External::BlinkerWithPwm<spa4, systemTimer, adap4, debug1>;
    using bsw5 = External::BlinkerWithPwm<spa5, systemTimer, adap5, debug1>;
    using bsw6 = External::BlinkerWithPwm<spa6, systemTimer, adap6, debug1>;
    using bsw7 = External::BlinkerWithPwm<spa7, systemTimer, adap7, debug1>;
#endif
	
    using bsws = Meta::List<bsw0, bsw1, bsw2, bsw3, bsw4, bsw5, bsw6, bsw7>;

	using patgen0 = void;
	using patgen1 = void;
	using patgen2 = void;
	using patgen3 = void;
	
    struct SwitchCallbackConfig;

    struct SwitchCallbackConfig {
        using debug = Devices::debug;
        using storage = Devices::storage;
        using bsws = Devices::bsws;
		using patgen0 = Devices::patgen0;
		using patgen1 = Devices::patgen1;
		using patgen2 = Devices::patgen2;
		using patgen3 = Devices::patgen3;
    };

    struct CrsfCallbackConfig {
        using debug = Devices::debug;
        using storage = Devices::storage;
        using bswList = bsws;
        using pwmList = Meta::List<pwm1, pwm3, pwm14, pwm17>;
        using timer = systemTimer;
        using crsf = Devices::crsf;
        using telemetry = Devices::telemetry;
        using switchCallback = SwitchCallback<SwitchCallbackConfig>;
		using patgen0 = Devices::patgen0;
		using patgen1 = Devices::patgen1;
		using patgen2 = Devices::patgen2;
		using patgen3 = Devices::patgen3;
        using slave = Devices::ssp;
    };
    struct CrsfConfig {
        using txpin = crsftx;
        using rxpin = txpin; // half-duplex
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRW  = csrfInDmaChannelComponent1;
        // using debug = void;
        using debug = Devices::debug;
        using tp = tp1;
        using callback = CrsfCallback<CrsfCallbackConfig>;
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

template<typename Config, typename MCU>
struct Devices<Nucleo, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using storage = Config::storage;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpioa, 9, MCU>;
    using crsfrx = Mcu::Stm::Pin<gpioa, 10, MCU>;

    // Usart 1: CRSF
    using csrfInDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
    using csrfInDmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma1::component_t, 2>;
    struct CrsfConfig;
    using crsf = RC::Protokoll::Crsf::V4::Master<1, CrsfConfig, MCU>;

#ifdef SERIAL_DEBUG
    using debugtx = Mcu::Stm::Pin<gpioa, 2, MCU>;
    struct DebugConfig;
    using debug = SerialBuffered<2, DebugConfig, MCU>;
    struct DebugConfig {
        using pin = debugtx;
        using clock = Devices::clock;
        static inline constexpr bool rxtxswap = false;
        static inline constexpr uint16_t bufferSize = 64;
    };
#else
    using debug = void;
#endif

    using adc = void;

    using telemetry = void;

    // Led
    using led = Mcu::Stm::Pin<gpioc, 6, MCU>;
    using ledBlinker = External::Blinker<led, systemTimer>;

    // Taster
#ifdef USE_BUTTON
    using button = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using btn = External::Button<button, systemTimer, External::Tick<systemTimer>{300ms}.raw(),
                                 External::Tick<systemTimer>{3000ms}.raw(), void>;
#else
    using btn = void;
#endif
    using in0 = void;
    using in1 = void;
#ifdef USE_TP1
    using tp1 = Mcu::Stm::Pin<gpiob, 4, MCU>;
#else
    using tp1 = void;
#endif
    using sw0 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using sw1 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using sw2 = Mcu::Stm::Pin<gpioa, 11, MCU>;
    using sw3 = Mcu::Stm::Pin<gpioa, 8, MCU>;
    using sw4 = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using sw5 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using sw6 = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using sw7 = Mcu::Stm::Pin<gpiob, 3, MCU>;

    using pwm3 = Mcu::Stm::V2::Pwm::Simple<3, clock>;
    using pwm1 = Mcu::Stm::V2::Pwm::Simple<1, clock>;
    using pwm14 = Mcu::Stm::V2::Pwm::Simple<14, clock>;
    using pwm2 = Mcu::Stm::V2::Pwm::Simple<2, clock>;

    // s0 : pa7  : tim3 ch2
    // s1 : pa6  : tim3 ch1
    // s2 : pa11 : tim1 ch4
    // (s3 : pa12 : isr)
    // s3 : pa8  : tim1 ch1
    // s4 : pa5  : tim2 ch1
    // s5 : pa4  : tim14 ch1
    // s6 : pa1  : tim2 ch2
    // (s7 : pa0  : isr)
    // s7 : pb3  : tim1 ch2

    using debug1 = void;
    using adap0 = Local::PwmAdapter<pwm3, 2, false, true, debug1>;
    using adap1 = Local::PwmAdapter<pwm3, 1, false, true, debug1>;
    using adap2 = Local::PwmAdapter<pwm1, 4, false, true, debug1>;
    using adap3 = Local::PwmAdapter<pwm1, 1, false, true, debug1>;
    using adap4 = Local::PwmAdapter<pwm2, 1, false, true, debug1>;
    using adap5 = Local::PwmAdapter<pwm14, 1, false, true, debug1>;
    using adap6 = Local::PwmAdapter<pwm2, 2, false, true, debug1>;
    using adap7 = Local::PwmAdapter<pwm1, 2, false, true, debug1>;

#ifdef USE_MORSE
    struct BConfig {
        using timer = systemTimer;
        using debug = void;
        static inline constexpr auto& text = storage::eeprom.morse_text;
    };
    struct BConfigDebug {
        using timer = systemTimer;
        using debug = Devices::debug;
        static inline constexpr auto& text = storage::eeprom.morse_text;
    };
    using bsw0 = External::Morse::BlinkerWithPwm<sw0, BConfig, adap0>;
    using bsw1 = External::Morse::BlinkerWithPwm<sw1, BConfig, adap1>;
    using bsw2 = External::Morse::BlinkerWithPwm<sw2, BConfig, adap2>;
    using bsw3 = External::Morse::BlinkerWithPwm<sw3, BConfig, adap3>;
    using bsw4 = External::Morse::BlinkerWithPwm<sw4, BConfig, adap4>;
    using bsw5 = External::Morse::BlinkerWithPwm<sw5, BConfig, adap5>;
    using bsw6 = External::Morse::BlinkerWithPwm<sw6, BConfig, adap6>;
    using bsw7 = External::Morse::BlinkerWithPwm<sw7, BConfig, adap7>;
#else
    using bsw0 = External::BlinkerWithPwm<sw0, systemTimer, adap0, debug1>;
    using bsw1 = External::BlinkerWithPwm<sw1, systemTimer, adap1, debug1>;
    using bsw2 = External::BlinkerWithPwm<sw2, systemTimer, adap2, debug1>;
    using bsw3 = External::BlinkerWithPwm<sw3, systemTimer, adap3, debug1>;
    using bsw4 = External::BlinkerWithPwm<sw4, systemTimer, adap4, debug1>;
    using bsw5 = External::BlinkerWithPwm<sw5, systemTimer, adap5, debug1>;
    using bsw6 = External::BlinkerWithPwm<sw6, systemTimer, adap6, debug1>;
    using bsw7 = External::BlinkerWithPwm<sw7, systemTimer, adap7, debug1>;
#endif

    using bsws = Meta::List<bsw0, bsw1, bsw2, bsw3, bsw4, bsw5, bsw6, bsw7>;

    struct SwitchCallbackConfig;

    struct SwitchCallbackConfig {
        using debug = Devices::debug;
        using storage = Devices::storage;
        using bsws = Devices::bsws;
    };

    struct CrsfCallbackConfig {
        using debug = Devices::debug;
        using storage = Devices::storage;
        using bswList = bsws;
        using pwmList = Meta::List<pwm1, pwm3, pwm14, pwm2>;
        using timer = systemTimer;
        using crsf = Devices::crsf;
        using telemetry = Devices::telemetry;
        using switchCallback = SwitchCallback<SwitchCallbackConfig>;
    };
    struct CrsfConfig {
        using txpin = crsftx;
#ifdef CRSF_HALFDUPLEX
        using rxpin = txpin;
#else
        using rxpin = crsfrx; // full-duplex
#endif
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfInDmaChannelComponent1;
        using dmaChWrite  = csrfInDmaChannelComponent2;
        // using debug = void;
        using debug = Devices::debug;
        using tp = tp1;
        using callback = CrsfCallback<CrsfCallbackConfig>;
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
#ifdef CRSF_TX_OPENDRAIN
        crsftx::template openDrain<true>();
        crsftx::template pullUp<true>();
#endif
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
        pwm1::init();
        pwm14::init();
        pwm2::init();
    }
};

template<typename Config, typename MCU>
struct Devices<WeAct, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using storage = Config::storage;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpioa, 9, MCU>;
    using crsfrx = Mcu::Stm::Pin<gpioa, 10, MCU>;

    // Usart 1: CRSF
    using csrfInDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
    using csrfInDmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma1::component_t, 2>;
    struct CrsfConfig;
    using crsf = RC::Protokoll::Crsf::V4::Master<1, CrsfConfig, MCU>;
    using crsfBuffer = crsf::messageBuffer;
	
#ifdef SERIAL_DEBUG
    using debugtx = Mcu::Stm::Pin<gpioa, 14, MCU>;
    struct DebugConfig;
    using debug = SerialBuffered<2, DebugConfig, MCU>;
    struct DebugConfig {
        using pin = debugtx;
        using clock = Devices::clock;
        static inline constexpr bool rxtxswap = true;
        static inline constexpr uint16_t bufferSize = 64;
    };
#else
    using debug = void;
#endif

    using adc = void;

    struct TelemConfig {
        using debug = Devices::debug;
        using timer = systemTimer;
        using clock = Devices::clock;
        using messagebuffer = crsfBuffer;
        using storage = Devices::storage;
    };
    using telemetry = Telemetry<TelemConfig>;

    // Led
    using led = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using invLed = Mcu::Stm::Gpio::Inverter<led>;
    using ledBlinker = External::Blinker<invLed, systemTimer>;

    // Taster
#ifdef USE_BUTTON
# ifdef SERIAL_DEBUG
#  undef USE_BUTTON
# else
    using button = Mcu::Stm::Pin<gpioa, 14, MCU>;
    using btn = External::Button<button, systemTimer, External::Tick<systemTimer>{300ms}.raw(),
                                 External::Tick<systemTimer>{3000ms}.raw(), void>;
# endif
#else
    using btn = void;
    using in0 = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using in1 = Mcu::Stm::Pin<gpiob, 7, MCU>;
#endif
#ifdef USE_TP1
    // using tp1 = Mcu::Stm::Pin<gpiob, 7, MCU>;
    using tp1 = void;
#else
    using tp1 = void;
#endif
    using sw0 = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using sw1 = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using sw2 = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using sw3 = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using sw4 = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using sw5 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using sw6 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using sw7 = Mcu::Stm::Pin<gpioa, 8, MCU>;

    struct MasterPwmConfig;
    struct SlavePwmConfig;

    using pwm2 = Mcu::Stm::V3::Pwm::Simple<2, MasterPwmConfig>;
    using pwm1 = Mcu::Stm::V3::Pwm::Simple<1, SlavePwmConfig>;
    using pwm3 = Mcu::Stm::V3::Pwm::Simple<3, SlavePwmConfig>;

    struct MasterPwmConfig {
        using clock = Devices::clock;
        static inline constexpr auto syncMode = Mcu::Stm::Timers::SyncMode::Master;
    };
    struct SlavePwmConfig {
        using clock = Devices::clock;
        static inline constexpr auto syncMode = Mcu::Stm::Timers::SyncMode::Slave;
        using master = pwm2;
    };

    // s0 : pa0  : tim2 ch1
    // s1 : pa1  : tim2 ch2
    // s2 : pa2  : tim2 ch3
    // s3 : pa3  : tim2 ch4
    // s4 : pb6  : tim1 ch3
    // s5 : pa6  : tim3 ch1
    // s6 : pa7  : tim3 ch2
    // s7 : pa8  : tim1 ch1

    using debug1 = void;
    using adap0 = Local::PwmAdapter<pwm2, 1, false, true, debug1>;
    using adap1 = Local::PwmAdapter<pwm2, 2, false, true, debug1>;
    using adap2 = Local::PwmAdapter<pwm2, 3, false, true, debug1>;
    using adap3 = Local::PwmAdapter<pwm2, 4, false, true, debug1>;
    using adap4 = Local::PwmAdapter<pwm1, 3, false, true, debug1>;
    using adap5 = Local::PwmAdapter<pwm3, 1, false, true, debug1>;
    using adap6 = Local::PwmAdapter<pwm3, 2, false, true, debug1>;
    using adap7 = Local::PwmAdapter<pwm1, 1, false, true, debug1>;

#ifdef USE_MORSE
    struct BConfig {
        using timer = systemTimer;
        using debug = void;
        static inline constexpr auto& text = storage::eeprom.morse_text;
    };
    struct BConfigDebug {
        using timer = systemTimer;
        using debug = Devices::debug;
        static inline constexpr auto& text = storage::eeprom.morse_text;
    };
    using bsw0 = External::Morse::BlinkerWithPwm<sw0, BConfig, adap0>;
    using bsw1 = External::Morse::BlinkerWithPwm<sw1, BConfig, adap1>;
    using bsw2 = External::Morse::BlinkerWithPwm<sw2, BConfig, adap2>;
    using bsw3 = External::Morse::BlinkerWithPwm<sw3, BConfig, adap3>;
    using bsw4 = External::Morse::BlinkerWithPwm<sw4, BConfig, adap4>;
    using bsw5 = External::Morse::BlinkerWithPwm<sw5, BConfig, adap5>;
    using bsw6 = External::Morse::BlinkerWithPwm<sw6, BConfig, adap6>;
    using bsw7 = External::Morse::BlinkerWithPwm<sw7, BConfig, adap7>;
#else
    using bsw0 = External::BlinkerWithPwm<sw0, systemTimer, adap0, debug1>;
    using bsw1 = External::BlinkerWithPwm<sw1, systemTimer, adap1, debug1>;
    using bsw2 = External::BlinkerWithPwm<sw2, systemTimer, adap2, debug1>;
    using bsw3 = External::BlinkerWithPwm<sw3, systemTimer, adap3, debug1>;
    using bsw4 = External::BlinkerWithPwm<sw4, systemTimer, adap4, debug1>;
    using bsw5 = External::BlinkerWithPwm<sw5, systemTimer, adap5, debug1>;
    using bsw6 = External::BlinkerWithPwm<sw6, systemTimer, adap6, debug1>;
    using bsw7 = External::BlinkerWithPwm<sw7, systemTimer, adap7, debug1>;
#endif

    using bsws = Meta::List<bsw0, bsw1, bsw2, bsw3, bsw4, bsw5, bsw6, bsw7>;

	struct PatGenConfig;
	using patgen0 = External::Pattern::Generator<0, PatGenConfig>;
	using patgen1 = void;
	using patgen2 = void;
	using patgen3 = void;
	
	struct PatGenConfig {
		using timer = systemTimer;
		using storage = Devices::storage;
        using debug = Devices::debug;
		using outputs = bsws;
		using messagebuffer = crsfBuffer;
	};
	
    struct SwitchCallbackConfig;

    struct SwitchCallbackConfig {
        using debug = Devices::debug;
        using storage = Devices::storage;
        using bsws = Devices::bsws;
		using patgen0 = Devices::patgen0;
		using patgen1 = Devices::patgen1;
		using patgen2 = Devices::patgen2;
		using patgen3 = Devices::patgen3;
    };

    struct CrsfCallbackConfig {
        using debug = Devices::debug;
        using storage = Devices::storage;
        using bswList = bsws;
        using pwmList = Meta::List<pwm1, pwm3, pwm2>;
        using timer = systemTimer;
        using crsf = Devices::crsf;
        using telemetry = Devices::telemetry;
        using switchCallback = SwitchCallback<SwitchCallbackConfig>;
		using patgen0 = Devices::patgen0;
		using patgen1 = Devices::patgen1;
		using patgen2 = Devices::patgen2;
		using patgen3 = Devices::patgen3;
    };
    struct CrsfConfig {
        using txpin = crsftx;
#ifdef CRSF_HALFDUPLEX
        using rxpin = txpin;
#else
        using rxpin = crsfrx; // full-duplex
#endif
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfInDmaChannelComponent1;
		using dmaChWrite  = csrfInDmaChannelComponent2;
        using dmaChRW  = csrfInDmaChannelComponent1;
        using debug = void;
        // using debug = Devices::debug;
        using tp = tp1;
        using callback = CrsfCallback<CrsfCallbackConfig>;
        static inline constexpr uint8_t fifoSize = 8;
    };
    static inline void init() {
        clock::init();

		SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA12_RMP; // PA10 (crsf rx)
        SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_RMP; // PA9 (crsf tx)
		
		systemTimer::init();

        gpioa::init();
        gpiob::init();
        gpioc::init();

        dma1::init();

        led::template dir<Mcu::Output>();
        ledBlinker::event(ledBlinker::Event::Off);

#ifdef USE_BUTTON
        btn::init();
#else
        in0::template dir<Mcu::Input>();
        in0::template pullup<true>();
        in1::template dir<Mcu::Input>();
        in1::template pullup<true>();
#endif
#ifdef USE_TP1
        tp1::template dir<Mcu::Output>();
#endif
        crsf::init();
#ifdef CRSF_TX_OPENDRAIN
        crsftx::template openDrain<true>();
        crsftx::template pullup<true>();
#endif

        sw0::template dir<Mcu::Output>();
        sw1::template dir<Mcu::Output>();
        sw2::template dir<Mcu::Output>();
        sw3::template dir<Mcu::Output>();
        sw4::template dir<Mcu::Output>();
        sw5::template dir<Mcu::Output>();
        sw6::template dir<Mcu::Output>();
        sw7::template dir<Mcu::Output>();

        pwm2::init();
        pwm3::init();
        pwm1::init();

        pwm2::start(); // synchron with pwm1 and pwm3;
    }
};
