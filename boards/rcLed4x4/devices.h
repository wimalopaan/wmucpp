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
#include "adc.h"
#include "usart.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "clock.h"
#include "watchdog.h"
#include "gpio.h"
#include "spi.h"
#include "tick.h"
#include "meta.h"
#include "rc/rc_2.h"
#include "rc/crsf_2.h"
#include "blinker.h"
#include "debug_2.h"
#include "button.h"

#include "pca9745.h"
#include "crsf_cb.h"
#include "telemetry.h"
#include "switch_cb.h"

struct Led01; // board: Led4x4
struct Led10; // board: Led4x4 (EasyEda, G0B1)

using namespace std::literals::chrono_literals;

template<typename HW, typename Config, typename MCU = DefaultMcu>
struct Devices;

template<typename Config, typename MCU>
struct Devices<Led10, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    struct WdgConfig {
        static inline constexpr uint32_t reload = 500;
    };
    using watchDog = WatchDog<WdgConfig>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using storage = Config::storage;

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;
    using csrfInDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
	using csrfInDmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma1::component_t, 2>;
    using spiDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 3>;
    // using spiDmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma1::component_t, 4>;
	using adcDmaChannel = Mcu::Components::DmaChannel<typename dma1::component_t, 5>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpioa, 9, MCU>;
	using crsfrx = Mcu::Stm::Pin<gpioa, 10, MCU>;

    // Usart 1: CRSF
    struct CrsfConfig;
    using crsf = RC::Protokoll::Crsf::V4::Master<1, CrsfConfig, MCU>;

#ifdef SERIAL_DEBUG
    // Usart 2: Debug
    using debugtx = Mcu::Stm::Pin<gpioa, 2, MCU>;
    struct DebugConfig;
    using debug = SerialBuffered<2, DebugConfig, MCU>;
    struct DebugConfig {
        using pin = debugtx;
        using clock = Devices::clock;
        static inline constexpr uint16_t bufferSize = 2048;
    };
#else
    using debug = void;
#endif

    // Led
    using led = Mcu::Stm::Pin<gpiob, 2, MCU>;
    using ledBlinker = External::Blinker<led, systemTimer>;

	using vin = Mcu::Stm::Pin<gpioa, 0, MCU>; // adc in0
    // ADC
    struct AdcConfig {
        using debug = void;
        using channels = std::integer_sequence<uint8_t, 0, 12>; // temp channel 12
        using dmaChannel = adcDmaChannel;
        using trigger = Mcu::Stm::ContinousSampling<256>;
        using isrConfig = Meta::List<>;
        static inline constexpr uint8_t slowChannel = 12; // temp
        static inline constexpr uint8_t slowSampleTime = 7;
    };
    using adc = Mcu::Stm::V4::Adc<1, AdcConfig>;
	
	static inline uint32_t r1 = 1'000;
    static inline uint32_t r2 = 12'000;
    static inline uint32_t Vref_n = 33;
    static inline uint32_t Vref_d = 10;
    static inline uint32_t Vcal_n = 30;

    struct TelemConfig {
        using debug = Devices::debug;
        using timer = systemTimer;
        using clock = Devices::clock;
        using messagebuffer = crsf::messageBuffer;
        using storage = Devices::storage;
    };
    using telemetry = Telemetry<TelemConfig>;
	
    // SPI 1: PCA
    using spi_cs = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using spi_miso = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using spi_mosi = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using spi_clk = Mcu::Stm::Pin<gpioa, 5, MCU>;

    using pca_oe = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using pca_reset = Mcu::Stm::Pin<gpiob, 1, MCU>;

    struct PcaConfig {
        using systemTimer = Devices::systemTimer;
        using debug = void;
        // using debug = Devices::debug;
        using cs = spi_cs;
        using miso = spi_miso;
        using mosi = spi_mosi;
        using clk = spi_clk;
        using oe = pca_oe;
        using reset = pca_reset;
        using txDmaComponent = spiDmaChannelComponent1;
        // using rxDmaComponent = spiDmaChannelComponent2;
    };
    using pca9745 = External::PCA9745<1, PcaConfig>;

	struct SwitchCallbackConfig {
		using pca = Devices::pca9745;
		using storage = Devices::storage;
	};
    struct CrsfCallbackConfig {
        using watchdog = Devices::watchDog;
        using timer = systemTimer;
        using crsf = Devices::crsf;
        using storage = Devices::storage;
        using pca = pca9745;
		using switchcallback = SwitchCallback<SwitchCallbackConfig>;
    };
    struct CrsfConfig {
        using txpin = crsftx;
        using rxpin = crsfrx; 
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfInDmaChannelComponent1;
		using dmaChWrite = csrfInDmaChannelComponent2;
        // using debug = void;
        using debug = Devices::debug;
        using tp = void;
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

        crsf::init();

        pca9745::init();

		adc::init();
        adc::oversample(7);
		
		adc::start();

        watchDog::init();
    }
};

template<typename Config, typename MCU>
struct Devices<Led01, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using storage = Config::storage;

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;
    using csrfInDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
    using spiDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 2>;
    // using spiDmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma1::component_t, 3>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpiob, 6, MCU>;

    // Usart 1: CRSF
    struct CrsfConfig;
    using crsf = RC::Protokoll::Crsf::V4::Master<1, CrsfConfig, MCU>;

#ifdef SERIAL_DEBUG
    // Usart 2: Debug
    using debugtx = Mcu::Stm::Pin<gpioa, 2, MCU>;
    struct DebugConfig;
    using debug = SerialBuffered<2, DebugConfig, MCU>;
    struct DebugConfig {
        using pin = debugtx;
        using clock = Devices::clock;
        static inline constexpr uint16_t bufferSize = 2048;
    };
#else
    using debug = void;
#endif

    // Led
    using led = Mcu::Stm::Pin<gpiob, 7, MCU>;
    using ledBlinker = External::Blinker<led, systemTimer>;

    // Taster
#ifdef USE_BUTTON
    using button = Mcu::Stm::Pin<gpioa, 11, MCU>;
    using btn = External::Button<button, systemTimer, External::Tick<systemTimer>{300ms}.raw(),
                                 External::Tick<systemTimer>{3000ms}.raw(), void>;
#endif

#ifdef USE_TP1
    using tp1 = Mcu::Stm::Pin<gpioa, 2, MCU>;
#else
    using tp1 = void;
#endif

    // SPI 1: PCA

    using spi_cs = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using spi_miso = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using spi_mosi = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using spi_clk = Mcu::Stm::Pin<gpioa, 1, MCU>;

    using pca_oe = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using pca_reset = Mcu::Stm::Pin<gpiob, 0, MCU>;

    struct PcaConfig {
        using systemTimer = Devices::systemTimer;
        using debug = void;
        // using debug = Devices::debug;
        using cs = spi_cs;
        using miso = spi_miso;
        using mosi = spi_mosi;
        using clk = spi_clk;
        using oe = pca_oe;
        using reset = pca_reset;
        using txDmaComponent = spiDmaChannelComponent1;
        // using rxDmaComponent = spiDmaChannelComponent2;
    };
    using pca9745 = External::PCA9745<1, PcaConfig>;

    using debug1 = void;

    struct CrsfCallbackConfig {
        using timer = systemTimer;
        using crsf = Devices::crsf;
        using storage = Devices::storage;
        using pca = pca9745;
        using switchcallback = struct swcb {
                static inline std::array<bool, 16> prevSwitchState{};
                static inline std::array<uint8_t, 4> groupCount{};
                static inline void setGroupIndex(const uint8_t index, const bool on) {
                    if (on) {
                        pca::groupStart(index);
                    }
                    else {
                        pca::groupStop(index);
                    }
                }
                static inline void setGroup(const uint8_t state8) {
                    for(uint8_t i = 0; i < 4; ++i) {
                        if (state8 & (1 << i)) {
                            pca::groupStart(i);
                        }
                        else {
                            pca::groupStop(i);
                        }
                    }
                }
                static inline void setVirtualIndex(const uint8_t i, const bool on) {
                    if (on) {
                        for(uint8_t k = 0; k < storage::eeprom.virtuals[i].member.size(); ++k) {
                            if (storage::eeprom.virtuals[i].member[k] >= 0) {
                                setIndex(storage::eeprom.virtuals[i].member[k], true);
                            }
                        }
                    }
                    else {
                        for(uint8_t k = 0; k < storage::eeprom.virtuals[i].member.size(); ++k) {
                            if (storage::eeprom.virtuals[i].member[k] >= 0) {
                                setIndex(storage::eeprom.virtuals[i].member[k], false);
                            }
                        }
                    }
                }
                static inline void setVirtual(const uint8_t state8) {
                    if (storage::eeprom.use_virtuals > 0) {
                        for(uint8_t i = 0; i < 8; ++i) {
                            if (state8 & (1 << i)) {
                                setVirtualIndex(i, true);
                            }
                            else {
                                setVirtualIndex(i, false);
                            }
                        }
                    }
                }
                static inline bool isMemberOfVirtual(const uint8_t i) {
                    for(uint8_t v = 0; v < storage::eeprom.virtuals.size(); ++v) {
                        for(uint8_t k = 0; k < storage::eeprom.virtuals[v].member.size(); ++k) {
                            if (i == storage::eeprom.virtuals[v].member[k]) {
                                return true;
                            }
                        }
                    }
                    return false;
                }
                static inline void set(const uint8_t state8) {
                    for(uint8_t i = 0; i < 8; ++i) {
                        const bool use = (storage::eeprom.use_virtuals == 0) || !isMemberOfVirtual(i);
                        if (use) {
                            if (state8 & (1 << i)) {
                                setIndex(i, true);
                            }
                            else {
                                setIndex(i, false);
                            }
                        }
                    }
                }
                static inline void set2(const uint8_t state8) {
                    for(uint8_t i = 0; i < 8; ++i) {
                        const bool use = (storage::eeprom.use_virtuals == 0) || !isMemberOfVirtual(i + 8);
                        if (use) {
                            if (state8 & (1 << i)) {
                                setIndex(i + 8, true);
                            }
                            else {
                                setIndex(i + 8, false);
                            }
                        }
                    }
                }
                static inline void setIndex(const uint8_t index, const bool on) {
                    if (on) {
                        if (storage::eeprom.outputs[index].groupStart > 0) {
                            const uint8_t group = storage::eeprom.outputs[index].group;
                            if ((group > 0) && !prevSwitchState[index]) {
                                if (storage::eeprom.groups[group - 1].mode == 0) {
                                    pca::groupStart(group - 1);
                                    groupCount[group - 1] += 1;
                                }
                                else {
                                    pca::groupHoldRampRestore(group - 1);
                                    pca::groupStart(group - 1);
                                }
                            }
                        }
                        pca::ledControl(index, storage::eeprom.outputs[index].control);
                        prevSwitchState[index] = true;
                    }
                    else {
                        pca::ledControl(index, 0);
                        if (storage::eeprom.outputs[index].groupStart > 0) {
                            const uint8_t group = storage::eeprom.outputs[index].group;
                            if ((group > 0) && prevSwitchState[index]) {
                                if (storage::eeprom.groups[group - 1].mode == 0) {
                                    groupCount[group - 1] -= 1;
                                    if (groupCount[group - 1] == 0) {
                                        pca::groupStop(group - 1);
                                    }
                                }
                                else {
                                    pca::groupStop(group - 1);
                                    pca::groupClear(group - 1);
                                }
                            }
                        }
                        prevSwitchState[index] = false;
                    }
                }
            };
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
#if defined(USE_TP1) && !defined(SERIAL_DEBUG)
        tp1::template dir<Mcu::Output>();
#endif
        crsf::init();

        pca9745::init();
    }
};

