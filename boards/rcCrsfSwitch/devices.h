#pragma once

#include <cstring>

#include "etl/fixedvector.h"
#include "etl/stackstring.h"

#include "meta.h"

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "components.h"
#include "dma.h"
#include "usart.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "clock.h"
#include "gpio.h"
#include "tick.h"
#include "meta.h"
#include "rc/rc.h"
#include "rc/crsf.h"

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

    using crsfInDmaChannel1 = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using crsfInDmaChannel2 = Mcu::Stm::Dma::Channel<dma1, 2, MCU>;

    // CRSF TX
    using crsftx = Mcu::Stm::Pin<gpioa, 9, MCU>; // AF 1
    // CRSF RX
    using crsfrx = Mcu::Stm::Pin<gpioa, 10, MCU>; // AF1
    // Usart 1: CRSF

    struct CrsfConfig {
        using Clock = clock;
        using ValueType = std::byte;
        static inline constexpr size_t size = std::max(RC::Protokoll::Crsf::maxMessageSize, uint8_t{64});
        using DmaChannelRead  = crsfInDmaChannel1;
        using DmaChannelWrite = crsfInDmaChannel2;
        static inline constexpr bool useRxToIsr = true;
        static inline constexpr uint16_t rxToCount = 20; // 2Bytes = 2(Start + 8 + S)
    };

    using crsf_in = Mcu::Stm::V2::Uart<1, CrsfConfig, MCU>;
    using uart1 = crsf_in;

    // LPUART 1: CRSF HD1
    using crsf_hd1_rxtx = Mcu::Stm::Pin<gpioa, 2, MCU>; // AF 6

    // USART 2: CRSF HD2
    using crsf_hd2_rxtx = Mcu::Stm::Pin<gpioa, 14, MCU>; // AF 1
    using tp0 = crsf_hd2_rxtx; // SWDIO Clk

    // USART 3: CRSF HD3
    using crsf_hd3_rxtx = Mcu::Stm::Pin<gpioa, 5, MCU>; // AF 4

    // USART 4: CRSF HD4
    using crsf_hd4_rxtx = Mcu::Stm::Pin<gpioa, 0, MCU>; // AF 4
    using debug   = Mcu::Stm::Uart<2, void, 1024, char, clock, MCU>;


    // USART 5: CRSF HD5
    using crsf_hd5_rxtx = Mcu::Stm::Pin<gpiob, 0, MCU>; // AF 8

    // USART 6: CRSF HD6
    using crsf_hd6_rxtx = Mcu::Stm::Pin<gpioa, 4, MCU>; // AF 3

    // LPUART 2: CRSF HD7
    using crsf_hd7_rxtx = Mcu::Stm::Pin<gpiob, 6, MCU>; // AF 10

    // Led
    using led = Mcu::Stm::Pin<gpiob, 9, MCU>;

    static inline void init() {
        clock::init();
        systemTimer::init();

        gpioa::init();
        gpiob::init();
        gpioc::init();

        led::template dir<Mcu::Output>();
        led::set();

        tp0::template dir<Mcu::Output>();

        debug::init();
        debug::baud(115200);
        crsf_hd4_rxtx::afunction(4);

        crsf_in::init();
        crsf_in::baud(420'000);
        crsftx::afunction(1);
        crsftx::template pullup<true>();
        crsfrx::afunction(1);

    }
};


