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
#include "debug_2.h"

#include "components.h"
#include "dma.h"
#include "dma_2.h"
#include "usart_2.h"
#include "i2c.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "clock.h"
#include "gpio.h"
#include "tick.h"
#include "meta.h"
#include "adc.h"
#include "blinker.h"
#include "rc/rc_2.h"
#include "rc/crsf_2.h"
#include "rc/sbus2_2.h"
#include "rc/spacemouse.h"
#include "encoder.h"

#include "crsf_cb.h"
#include "pcal6408.h"
#include "switches.h"
#include "hwext.h"
#include "eeprom.h"

struct SW01;
struct Desk01;

template<typename HW, typename Config, typename MCU = DefaultMcu>
struct Devices;

template<typename Config, typename MCU>
struct Devices<Desk01, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using auxes1 = Config::auxes1;
    using auxes2 = Config::auxes2;

    using smes1 = Config::smes1;
    using smes2 = Config::smes2;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiod = Mcu::Stm::GPIO<Mcu::Stm::D, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;
    using dma2 = Mcu::Stm::Dma::Controller<2, MCU>;

    using storage = Config::storage;

    // Ubersicht: Pins

    using led1 = Mcu::Stm::Pin<gpioc, 13, MCU>;
    using led2 = Mcu::Stm::Pin<gpiod, 0, MCU>;

    // using bus_tx = Mcu::Stm::Pin<gpiof, 2, MCU>; // also reset
    using bus_rx = Mcu::Stm::Pin<gpioc, 7, MCU>;
    using tp = bus_rx;
    using sm2_tx = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using sm2_rx = Mcu::Stm::Pin<gpioa, 1, MCU>;

    using ana1 = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using ana2 = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using ana3 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using ana4 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using ana5 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using ana6 = Mcu::Stm::Pin<gpiob, 1, MCU>;

    using incr2_a = Mcu::Stm::Pin<gpioa, 5, MCU>; // tim2_ch1 (af2)
    using incr2_b = Mcu::Stm::Pin<gpiob, 3, MCU>; // tim2_ch2 (af2)
    using incr2_t = Mcu::Stm::Pin<gpiod, 1, MCU>;

    using incr1_a = Mcu::Stm::Pin<gpioc, 6, MCU>; // tim3_ch1 (af1)
    using incr1_b = Mcu::Stm::Pin<gpiob, 5, MCU>; // tim3_ch2 (af1)
    using incr1_t = Mcu::Stm::Pin<gpiob, 4, MCU>;

    using aux2_rx = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using aux2_tx = Mcu::Stm::Pin<gpiob, 2, MCU>;

    using button1 = Mcu::Stm::Pin<gpiob, 10, MCU>;
    using button2 = Mcu::Stm::Pin<gpiob, 12, MCU>;

    using debug_tx = Mcu::Stm::Pin<gpiob, 11, MCU>;

    using crsf_rx = Mcu::Stm::Pin<gpiob, 9, MCU>;
    using crsf_tx = Mcu::Stm::Pin<gpiob, 8, MCU>;

    using i2c2_scl = Mcu::Stm::Pin<gpiob, 13, MCU>;
    using i2c2_sda = Mcu::Stm::Pin<gpiob, 14, MCU>;

    using bt_en = Mcu::Stm::Pin<gpiob, 15, MCU>;
    using bt_pwr = Mcu::Stm::Pin<gpioa, 8, MCU>;
    using bt_tx = Mcu::Stm::Pin<gpioa, 9, MCU>;
    using bt_rx = Mcu::Stm::Pin<gpioa, 10, MCU>;
    using bt_status = Mcu::Stm::Pin<gpioa, 11, MCU>;

#ifndef USE_SWD
    using aux1_tx = Mcu::Stm::Pin<gpioa, 14, MCU>;
    using aux1_rx = Mcu::Stm::Pin<gpioa, 15, MCU>;
#endif

    using sm1_rx = Mcu::Stm::Pin<gpiod, 2, MCU>;
    using sm1_tx = Mcu::Stm::Pin<gpiod, 3, MCU>;

    using i2c1_scl = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using i2c1_sda = Mcu::Stm::Pin<gpiob, 7, MCU>;

    // Uebersicht: DMA
    // crsf (RX)
    using csrfDmaChannel1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
    using csrfDmaChannel2 = Mcu::Components::DmaChannel<typename dma1::component_t, 2>;
    // adc
    using adcDmaChannel = Mcu::Components::DmaChannel<typename dma1::component_t, 3>;
    // half-duplex
    using aux1DmaChannel = Mcu::Components::DmaChannel<typename dma1::component_t, 4>;
    using aux2DmaChannel = Mcu::Components::DmaChannel<typename dma1::component_t, 5>;

    using sm1DmaChannel = Mcu::Components::DmaChannel<typename dma1::component_t, 6>;
    using sm2DmaChannel = Mcu::Components::DmaChannel<typename dma1::component_t, 7>;

    // ADC
    struct AdcConfig {
        using channels = std::integer_sequence<uint8_t, 2, 3, 4, 6, 7, 9>;
        using dmaChannel = adcDmaChannel;
        using trigger = Mcu::Stm::ContinousSampling<2>;
        using isrConfig = Meta::List<Mcu::Stm::EndOfSequence>;
    };
    using adc = Mcu::Stm::V4::Adc<1, AdcConfig>;

    // USARTS
    // Usart 2: radio aux1
#ifndef USE_SWD
    struct Aux1Config;
    using hwext1 = HwExtension<2, Aux1Config, MCU>;
    struct SBus1Config;
    using sbus1 = RC::Protokoll::SBus2::V4::Master<2, SBus1Config, MCU>;
#endif

    // Usart 3: radio aux2

    struct Aux2Config;
    using hwext2 = HwExtension<3, Aux2Config, MCU>;
    struct SBus2Config;
    using sbus2 = RC::Protokoll::SBus2::V4::Master<3, SBus2Config, MCU>;

    // Usart 4: spacemouse 2

    struct SM2Config;
    using sm2 = RC::Protokoll::SpaceMouse::Input<4, SM2Config, MCU>;

    // Usart 5: spacemouse 1

    struct SM1Config;
    using sm1 = RC::Protokoll::SpaceMouse::Input<5, SM1Config, MCU>;

    // Usart 6: ELRX receiver
    struct CrsfConfig;
    using crsf_in = RC::Protokoll::Crsf::V4::Master<6, CrsfConfig, MCU>;



#ifdef SERIAL_DEBUG
    // LPUart1
    struct DebugConfig;
    using debug = SerialBuffered<101, DebugConfig>;
    struct DebugConfig {
        static inline constexpr uint16_t bufferSize = 1024;
        using pin = debug_tx;
        using clock = Devices::clock;
    };
#else
    using debug = void;
#endif

    using ledBlinker1 = External::Blinker<led1, systemTimer>;
    using ledBlinker2 = External::Blinker<led2, systemTimer>;

    struct Enc1Config {
        using pin_a = incr1_a;
        using pin_b = incr1_b;
        using pin_t = incr1_t;
        static inline constexpr uint32_t max = 255;
    };
    using enc1 = External::Encoder<3, Enc1Config>;

    struct Enc2Config {
        using pin_a = incr2_a;
        using pin_b = incr2_b;
        using pin_t = incr2_t;
        static inline constexpr uint32_t max = 255;
    };
    using enc2 = External::Encoder<2, Enc2Config>;

    // I2C-1
    struct I2C1Config {
        using sda_pin = i2c1_sda;
        using scl_pin = i2c1_scl;
        static inline constexpr size_t size = 16;
        using debug = Devices::debug;
    };
    using i2c1 = Mcu::Stm::I2C::V2::Master<1, I2C1Config>;

    // I2C-2
    struct I2C2Config {
        using sda_pin = i2c2_sda;
        using scl_pin = i2c2_scl;
        static inline constexpr size_t size = 16;
        using debug = Devices::debug;
    };
    using i2c2 = Mcu::Stm::I2C::V2::Master<2, I2C2Config>;

    static inline constexpr Mcu::Stm::I2C::Address pcaAdr0{0x20};
    static inline constexpr Mcu::Stm::I2C::Address pcaAdr1{0x21};
    using pca0 = External::V2::PCAL6408<i2c1, pcaAdr1, systemTimer>;
    using pca1 = External::V2::PCAL6408<i2c1, pcaAdr0, systemTimer>;
    using pca2 = External::V2::PCAL6408<i2c2, pcaAdr1, systemTimer>;
    using pca3 = External::V2::PCAL6408<i2c2, pcaAdr0, systemTimer>;

    struct Sw1Config {
        using timer = systemTimer;
        using pcas = Meta::List<pca0, pca1>;
        using debug = Devices::debug;
        using callback = struct {
            static inline void set(const uint8_t index, const bool state) {
                hwext1::set(index, state);
                hwext2::set(index, state);
            }
        };
    };
    using switches1 = External::Switches<Sw1Config>;
    struct Sw2Config {
        using timer = systemTimer;
        using pcas = Meta::List<pca2, pca3>;
        using debug = Devices::debug;
        using callback = struct {
            static inline void set(const uint8_t index, const bool state) {
                hwext1::set(index + 32, state);
                hwext2::set(index + 32, state);
            }
        };
    };
    using switches2 = External::Switches<Sw2Config>;

#ifndef USE_SWD
    struct Aux1Config {
        using clock = Devices::clock;
        using systemTimer = Devices::systemTimer;
        using dmaChComponent = aux1DmaChannel;
        using pin = aux1_tx;
        using input = crsf_in::input;
        using storage = Devices::storage;
        using debug = Devices::debug;
        using tp = void;
    };
    struct SBus1Config {
        using clock = Devices::clock;
        using debug = Devices::debug;
        using dmaChComponent = aux1DmaChannel;
        using systemTimer = Devices::systemTimer;
        using adapter = void;
        using pin = aux1_tx;
        using tp = void;
    };
#endif
    struct Aux2Config {
        using clock = Devices::clock;
        using systemTimer = Devices::systemTimer;
        using dmaChComponent = aux2DmaChannel;
        using pin = aux2_tx;
        using input = crsf_in::input;
        using storage = Devices::storage;
        using debug = Devices::debug;
        using tp = void;
    };
    struct SBus2Config {
        using clock = Devices::clock;
        using debug = Devices::debug;
        using dmaChComponent = aux2DmaChannel;
        using systemTimer = Devices::systemTimer;
        using adapter = void;
        using pin = aux2_tx;
        using tp = void;
    };
    struct SM1Config {
        using clock = Devices::clock;
        using debug = Devices::debug;
        using dmaChComponent = sm1DmaChannel;
        using systemTimer = Devices::systemTimer;
        using adapter = void;
        using pin_rx = sm1_rx;
        using pin_tx = sm1_tx;
        using tp = void;
    };
    struct SM2Config {
        using clock = Devices::clock;
        using debug = Devices::debug;
        using dmaChComponent = sm2DmaChannel;
        using systemTimer = Devices::systemTimer;
        using adapter = void;
        using pin_rx = sm2_rx;
        using pin_tx = sm2_tx;
        using tp = void;
    };

    struct CrsfCallbackConfig {
        using storage = Config::storage;
        using timer = systemTimer;
        using src = crsf_in;
        using auxes1 = Devices::auxes1;
        using auxes2 = Devices::auxes2;
        using smes1 = Devices::smes1;
        using smes2 = Devices::smes2;
        using tp = void;
        using debug = Devices::debug;
    };
    struct CrsfConfig {
        using rxpin = crsf_rx;
        using txpin = crsf_tx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfDmaChannel1;
        using dmaChWrite = csrfDmaChannel2;
        using debug = Devices::debug;
        using tp = void;
        using callback = CrsfCallback<CrsfCallbackConfig, debug>;
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
        gpiod::init();
        gpiof::init();

        led1::template dir<Mcu::Output>();
        led2::template dir<Mcu::Output>();

        tp::template dir<Mcu::Output>();

        button1::template dir<Mcu::Input>();
        button2::template dir<Mcu::Input>();
        button1::pullup();
        button2::pullup();

        enc1::init();
        enc2::init();


#ifdef SERIAL_DEBUG
        debug::init();
#endif

        crsf_in::init();

        adc::init();
        adc::oversample(8); // 256
        adc::start();

        i2c1::init();
        i2c2::init();

        sm1::init();
        sm2::init();

        bt_pwr::template dir<Mcu::Output>();
        bt_pwr::reset();

    }
};



template<typename Config, typename MCU>
struct Devices<SW01, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;
    using dma2 = Mcu::Stm::Dma::Controller<2, MCU>;

    using storage = Config::storage;

    // Ubersicht: Pins

    // Tlm2: PB9 : TIM17-CH1, TIM4-CH4
    using tp1 = Mcu::Stm::Pin<gpiob, 9, MCU>;

    // Uebersicht: DMA

    // full-duplex: uart1
    using csrfInDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
    using csrfInDmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma1::component_t, 2>;
    // adc
    // using adcDmaChannel     = Mcu::Stm::Dma::Channel<dma1, 3, MCU>;
    // half-duplex
    using esc1DmaChannelComponent = Mcu::Components::DmaChannel<typename dma2::component_t, 5>;
    // using esc2DmaChannelComponent = Mcu::Components::DmaChannel<typename dma2::component_t, 4>;

    // Uebersicht: UART
    // Uart 1: CRSF-IN
    // Uart 2: ESC1
    // Uart 3: ESC2
    // Uart 4: CRSF-FD, GPS, AUX
    // Uart 5: Srv1
    // Uart 6: Srv2
    // LPUart 1: Debug
    // LPUart 2: CRSF-HD, SBus(2)

    // Usart 1: CRSF
    using crsftx = Mcu::Stm::Pin<gpioa, 9, MCU>; // AF1
    using crsfrx = Mcu::Stm::Pin<gpioa, 10, MCU>; // AF1

    struct CrsfConfig;
    using crsf_in = RC::Protokoll::Crsf::V4::Master<1, CrsfConfig, MCU>;

    // Usart 2: Radio
    using esc1_pin = Mcu::Stm::Pin<gpioa, 2, MCU>;
    struct SerialConfig1;
    using radio = HwExtension<2, SerialConfig1, MCU>;

    // debug auf LPUART1 (PA3 AF(6), RX<->TX tauschen) : Telemetry-1
#ifdef SERIAL_DEBUG
    using debugrx = Mcu::Stm::Pin<gpioa, 3, MCU>;
    struct DebugConfig;
    using debug = SerialBuffered<101, DebugConfig, MCU>;
    struct DebugConfig {
        static inline constexpr uint16_t bufferSize = 1024;
        static inline constexpr bool rxtxswap = true;
        using pin = debugrx;
        using clock = Devices::clock;
    };
#else
    using debug = void;
#endif

    // Led
    using led1 = Mcu::Stm::Pin<gpiob, 7, MCU>;
    using led2 = Mcu::Stm::Pin<gpiob, 8, MCU>;

    using ledBlinker1 = External::Blinker<led1, systemTimer>;
    using ledBlinker2 = External::Blinker<led2, systemTimer>;

    // Uart4: CRSF-FD / AUX
    using auxrx = Mcu::Stm::Pin<gpioa, 1, MCU>; // AF4
    using auxtx = Mcu::Stm::Pin<gpioa, 0, MCU>; // AF4

    // I2C-3
    using sda3 = Mcu::Stm::Pin<gpiob, 4, MCU>;
    using scl3 = Mcu::Stm::Pin<gpiob, 3, MCU>;
    using i2c3 = Mcu::Stm::I2C::Master<3, 16, debug, MCU>;

    static inline constexpr Mcu::Stm::I2C::Address pcaAdr0{0x20};
    static inline constexpr Mcu::Stm::I2C::Address pcaAdr1{0x21};
    using pca0 = External::PCAL6408<i2c3, pcaAdr1, systemTimer>;
    using pca1 = External::PCAL6408<i2c3, pcaAdr0, systemTimer>;

    struct SerialConfig1 {
        using clock = Devices::clock;
        using systemTimer = Devices::systemTimer;
        using dmaChComponent = esc1DmaChannelComponent;
        using pin = esc1_pin;
        using input = crsf_in::input;
        using storage = Devices::storage;
        using debug = void;
        using tp = void;
    };
    struct CrsfCallbackConfig {
        using storage = Config::storage;
        using timer = systemTimer;
        using src = crsf_in;
        using radio = Devices::radio;
        using tp = void;
        using debug = Devices::debug;
    };
    struct CrsfConfig {
        using rxpin = crsfrx;
        using txpin = crsftx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfInDmaChannelComponent1;
        using dmaChWrite = csrfInDmaChannelComponent2;
        using debug = Devices::debug;
        using tp = void;
        using callback = CrsfCallback<CrsfCallbackConfig, debug>;
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

#ifdef SERIAL_DEBUG
        debug::init();
#endif

        crsf_in::init();

        sda3::openDrain();
        scl3::openDrain();
        sda3::afunction(6);
        scl3::afunction(6);
        i2c3::init();

        // adc::init();
        // adc::oversample(8); // 256

        tp1::template dir<Mcu::Output>();
        // tp3::template dir<Mcu::Output>();
    }
};


