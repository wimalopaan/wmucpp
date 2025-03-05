#pragma once

#include <cstring>

#include "meta.h"

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "debug_2.h"

#include "components.h"
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

#include "pcal6408.h"
#include "switches.h"

#include "eeprom.h"

struct SW01;

template<typename HW, typename Config, typename MCU = DefaultMcu>
struct Devices;

template<typename Config, typename MCU>
struct Devices<SW01, Config, MCU> {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;
    using dma2 = Mcu::Stm::Dma::Controller<2, MCU>;

    // Ubersicht: Pins

    // Uebersicht: DMA

    // full-duplex: uart1
    using csrfInDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
    using csrfInDmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma1::component_t, 2>;
    // adc
    // using adcDmaChannel     = Mcu::Stm::Dma::Channel<dma1, 3, MCU>;
    // half-duplex
    using srv1DmaChannelComponent = Mcu::Components::DmaChannel<typename dma1::component_t, 4>;
    // I2C
    // using i2cDmaChannel     = Mcu::Stm::Dma::Channel<dma1, 5, MCU>;

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

        sda3::openDrain();
        scl3::openDrain();
        sda3::afunction(6);
        scl3::afunction(6);
        i2c3::init();

        // adc::init();
        // adc::oversample(8); // 256

        // tp1::template dir<Mcu::Output>();
        // tp3::template dir<Mcu::Output>();
    }
};


