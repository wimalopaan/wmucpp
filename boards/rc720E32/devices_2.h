#pragma once

#include <cstring>

#include "etl/fixedvector.h"
#include "etl/stackstring.h"

#include "meta.h"

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "debug_2.h"

#include "components.h"
#include "dma.h"
#include "dma_2.h"
#include "usart_2.h"
#include "usart.h"
#include "i2c.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "clock.h"
#include "gpio.h"
#include "tick.h"
#include "meta.h"
#include "rc/rc_2.h"
#include "rc/crsf_2.h"
#include "rc/escape_2.h"
#include "rc/sbus2_2.h"
#include "rc/vesc_2.h"
#include "pwm.h"
#include "adc.h"
#include "blinker.h"
#include "adapter.h"
#include "crsf_cb_2.h"
#include "eeprom.h"
#include "rc/waveshare_2.h"
#include "fbservo.h"
#include "polar.h"
#include "channels.h"
#include "package_relay_2.h"
#include "telemetry_2.h"
#include "iservo.h"
#include "pulse_input.h"
#include "inputmapper.h"
#include "rc/ibus_2.h"
#include "rc/sbus_2.h"
#include "rc/sumdv3_2.h"
#include "crsfchannelcollector.h"

struct SW01;

template<typename HW, typename Config, typename MCU = DefaultMcu>
struct Devices;

template<typename Config, typename MCU>
struct Devices<SW01, Config, MCU> {
    using storage = Config::storage;

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
    using crsfInDmaChannel1 = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using csrfInDmaChannelComponent1 = Mcu::Components::DmaChannel<typename dma1::component_t, 1>;
    using crsfInDmaChannel2 = Mcu::Stm::Dma::Channel<dma1, 2, MCU>;
    using csrfInDmaChannelComponent2 = Mcu::Components::DmaChannel<typename dma1::component_t, 2>;
    // adc
    using adcDmaChannel     = Mcu::Stm::Dma::Channel<dma1, 3, MCU>;
    // half-duplex
    using srv1DmaChannel    = Mcu::Stm::Dma::Channel<dma1, 4, MCU>;
    using srv1DmaChannelComponent = Mcu::Components::DmaChannel<typename dma1::component_t, 4>;
    // I2C
    using i2cDmaChannel     = Mcu::Stm::Dma::Channel<dma1, 5, MCU>;
    // half-duplex
    using sbus1DmaChannel   = Mcu::Stm::Dma::Channel<dma1, 6, MCU>;
    using sbus1DmaChannelComponent = Mcu::Components::DmaChannel<typename dma1::component_t, 6>;
    // input capture
    using pulseInDmaChannel = Mcu::Stm::Dma::Channel<dma1, 7, MCU>;
    // half-duplex
    using relay1DmaChannel  = Mcu::Stm::Dma::Channel<dma2, 1, MCU>;
    using relay1DmaChannelComponent = Mcu::Components::DmaChannel<typename dma2::component_t, 1>;
    // half-duplex
    using relayAuxDmaChannel= Mcu::Stm::Dma::Channel<dma2, 2, MCU>;
    using relayAuxDmaChannelComponent = Mcu::Components::DmaChannel<typename dma2::component_t, 2>;
    // half-duplex
    using srv2DmaChannel    = Mcu::Stm::Dma::Channel<dma2, 3, MCU>;
    using srv2DmaChannelComponent = Mcu::Components::DmaChannel<typename dma2::component_t, 3>;
    // half-duplex
    using esc2DmaChannel    = Mcu::Stm::Dma::Channel<dma2, 4, MCU>;
    using esc2DmaChannelComponent = Mcu::Components::DmaChannel<typename dma2::component_t, 4>;
    // half-duplex
    using esc1DmaChannel    = Mcu::Stm::Dma::Channel<dma2, 5, MCU>;
    using esc1DmaChannelComponent = Mcu::Components::DmaChannel<typename dma2::component_t, 5>;

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
    using crsfBuffer = crsf_in::messageBuffer;

    // debug auf LPUART1 (PA3 AF(6), RX<->TX tauschen) : Telemetry-1
#ifdef SERIAL_DEBUG
    using debugrx = Mcu::Stm::Pin<gpioa, 3, MCU>;
#ifdef USE_UART_2
    struct DebugConfig;
    using debug = SerialBuffered<101, DebugConfig, MCU>;
    struct DebugConfig {
        static inline constexpr uint16_t bufferSize = 1024;
        static inline constexpr bool rxtxswap = true;
        using pin = debugrx;
        using clock = Devices::clock;
    };
#else
    using debug = Mcu::Stm::V1::LpUart<1, void, 1024, char, clock, MCU>;
#endif
#else
    using debug = void;
#endif

    struct InputConfig;
    using inputs = InputMapper<InputConfig>;

    using polar1 = Polar<0, inputs, typename Config::storage>;
    using polar2 = Polar<1, inputs, typename Config::storage>;

    // Led
    using led1 = Mcu::Stm::Pin<gpiob, 7, MCU>;
    using led2 = Mcu::Stm::Pin<gpiob, 8, MCU>;

    using ledBlinker1 = External::Blinker<led1, systemTimer>;
    using ledBlinker2 = External::Blinker<led2, systemTimer>;

    // Übersicht: Timer <-> Pins
    // Channel
    //          Channel1    Channel2    Channel3    Channel4
    // -------------------------------------------------------
    // TIM1
    // TIM2                             Esc1-Out    Esc1-Tlm
    // TIM3     Srv1-Fb     Srv2-Fb     Srv1-Out
    //          (cppm-in)
    // TIM4     cppm-in                             Esc2-Tlm
    // TIM14    Srv2-Out
    // TIM15    Esc1-Out    Esc1-Tlm
    // TIM16    Srv1-Fb
    // TIM17    Esc2-Out

    // ESC1: PA2 : Uart2-TX (AF1), TIM2-CH3 (AF2), TIM15-CH1 (AF5)
    using esc1_pin = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using pwm2 = Mcu::Stm::V2::Pwm::Servo<2, clock>;
    using esc1_pwm = PwmAdapter<pwm2, esc1_pin, 3, storage, std::integral_constant<uint8_t, 0>, debug>;

    struct SerialConfig1;
#ifdef USE_UART_2
    using esc32_1 = RC::Protokoll::ESCape::V2::Serial<2, SerialConfig1, MCU>;
    using esc32ascii_1 = RC::Protokoll::ESCape::V2::ConfigAscii<2, SerialConfig1, MCU>;
#else
    using esc32_1 = RC::ESCape::Serial<2, SerialConfig1, MCU>;
    using esc32ascii_1 = RC::ESCape::ConfigAscii<2, SerialConfig1, MCU>;
#endif

    struct VEscConfig1;
    using vesc_1 = RC::VESC::Master::V5::Serial<2, VEscConfig1, MCU>;

    // Tlm1: PA3 : Uart2-RX (AF1), TIM2-CH4 (AF2), TIM15-CH2 (AF5)

    // Srv1: PB0 : TIM3-CH3 (AF1), TIM1-CH2N(AF2), Uart3-RX (AF4), Uart5-TX (AF8)
    using srv1_pin = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using pwm3 = Mcu::Stm::V2::Pwm::Servo<3, clock>;
    using srv1_pwm = PwmAdapter<pwm3, srv1_pin, 3, storage, void>;

    struct WS1Config;
#ifdef USE_UART_2
    using srv1_waveshare = External::WaveShare::V2::Servo<5, WS1Config, MCU>;
#else
    using srv1_waveshare = WaveShare<5, WS1Config, MCU>;
#endif
    // Fb1:  PA6 : TIM3-CH1 (AF1), TIM16-CH1(AF5), ADC-IN6
    using fb1_pin = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using adc = Mcu::Stm::V3::Adc<1, Meta::NList<6, 5>, Mcu::Stm::ContinousSampling<16>, adcDmaChannel, std::array<uint16_t, 2>, Meta::List<Mcu::Stm::EndOfSequence>>;
    using srv1_fb = FeedbackAdapter<0, adc, fb1_pin, debug>;
    using srv1_feetech = Feetech<0, polar1, srv1_fb, srv1_pwm, systemTimer, debug>;

    // Fehler auf Platine
    // ESC2: PB2 : Uart3-TX (AF4)
    // verbinden mit PA8 : TIM1-CH1 (AF2): Uart1-TX PA9 erscheint als OpenDrain hier???
    // verbinden mit PA7 : TIM17-CH1 (AF5)
    using esc2_pin_1 = Mcu::Stm::Pin<gpiob, 2, MCU>;
    using esc2_pin = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using pwm17 = Mcu::Stm::V2::Pwm::Servo<17, clock>;
    using esc2_pwm = PwmAdapter<pwm17, esc2_pin, 1, storage, std::integral_constant<uint8_t, 1>, debug>;

    struct SerialConfig2;
#ifdef USE_UART_2
    using esc32_2 = RC::Protokoll::ESCape::V2::Serial<3, SerialConfig2, MCU>;
    using esc32ascii_2 = RC::Protokoll::ESCape::V2::ConfigAscii<3, SerialConfig2, MCU>;
#else
    using esc32_2 = RC::ESCape::Serial<3, SerialConfig2, MCU>;
    using esc32ascii_2 = RC::ESCape::ConfigAscii<3, SerialConfig2, MCU>;
#endif

    struct VEscConfig2;
    using vesc_2 = RC::VESC::Master::V5::Serial<3, VEscConfig2, MCU>;

    // Tlm2: PB9 : TIM17-CH1, TIM4-CH4
    using tp1 = Mcu::Stm::Pin<gpiob, 9, MCU>;

    // Srv2: PA4 : TIM14-CH1 (AF4), Uart6-TX (AF3)
    using srv2_pin = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using pwm14 = Mcu::Stm::V2::Pwm::Servo<14, clock>;
    using srv2_pwm = PwmAdapter<pwm14, srv2_pin, 1, storage, void>;

    struct WS2Config;
#ifdef USE_UART_2
    using srv2_waveshare = External::WaveShare::V2::Servo<6, WS2Config, MCU>;
#else
    using srv2_waveshare = WaveShare<6, WS2Config, MCU>;
#endif
    // Fehler auf Platine (PWM Messung): mit PB5 (TIM3-CH2) verbinden
    // Fb2:  PA5 : ADC-IN5
    using tp3 = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using fb2_pin = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using srv2_fb = FeedbackAdapter<1, adc, fb2_pin>;
    using srv2_feetech = Feetech<1, polar2, srv2_fb, srv2_pwm, systemTimer, debug>;

    // Uart4: CRSF-FD / AUX
    using auxrx = Mcu::Stm::Pin<gpioa, 1, MCU>; // AF4
    using auxtx = Mcu::Stm::Pin<gpioa, 0, MCU>; // AF4

#ifdef USE_UART_2
    struct RelayAuxConfig;
    using relay_aux = RC::Protokoll::Crsf::V4::PacketRelay<4, RelayAuxConfig, MCU>;
#else
    struct RelayDebug;
    using relay_aux = PacketRelay<4, true, auxtx, crsf_in, crsfBuffer, relayAuxDmaChannel, systemTimer, clock, RelayDebug, MCU>;
#endif

    // LPUart2: Sbus, CRSF-HD
    using sbus_crsf_pin = Mcu::Stm::Pin<gpioc, 6, MCU>;
    struct SBus1Config;
#ifdef USE_UART_2
    using sbus1 = RC::Protokoll::SBus2::V4::Master<102, SBus1Config, MCU>;
#else
    using sbus1 = RC::Protokoll::SBus2::V3::Master<102, SBus1Config, MCU>;
#endif

    struct RelayConfig;
#ifdef USE_UART_2
    using relay1 = RC::Protokoll::Crsf::V4::PacketRelay<102, RelayConfig, MCU>;
#else
    using relay1 = PacketRelay<102, true, sbus_crsf_pin, crsf_in, crsfBuffer, relay1DmaChannel, systemTimer, clock, RelayDebug, MCU>;
#endif

    struct IBusConfig;
#ifdef USE_UART_2
    using ibus_in = RC::Protokoll::IBus::V2::Input<102, IBusConfig, MCU>;
#else
    using ibus_in = IBusInput<102, IBusConfig, MCU>;
#endif

    struct SBusConfig;
#ifdef USE_UART_2
    using sbus_in = RC::Protokoll::SBus::V2::Input<102, SBusConfig, MCU>;
#else
    using sbus_in = SBusInput<102, SBusConfig, MCU>;
#endif

    struct Sumdv3Config;
#ifdef USE_UART_2
    using sumdv3_in = RC::Protokoll::SumDV3::V2::Input<102, Sumdv3Config, MCU>;
#else
    using sumdv3_in = SumDV3Input<102, Sumdv3Config, MCU>;
#endif

    using pulse_pin = Mcu::Stm::Pin<gpiob, 6, MCU>;
    struct PulseConfig;
    using pulse_in = Pulse::CppmIn<4, PulseConfig, MCU>; // TIM4-CH1

    // I2C-3

    using sda3 = Mcu::Stm::Pin<gpiob, 4, MCU>;
    using scl3 = Mcu::Stm::Pin<gpiob, 3, MCU>;
    using i2c = Mcu::Stm::I2C::Master<3, 16, debug, MCU>;

    struct RelayConfig {
        using pin = sbus_crsf_pin;
        using clock = Devices::clock;
        using systemTimer = Devices::systemTimer;
#ifdef USE_UART_2
        using dmaChComponent = relay1DmaChannelComponent;
#else
        using dmaChRead = relay1DmaChannel;
#endif
        using debug = void;
        using tp = tp1;
        using src = crsf_in::input;
        using dest = crsfBuffer;
    };
    struct RelayAuxConfig {
        using pin = auxtx;
        using clock = Devices::clock;
        using systemTimer = Devices::systemTimer;
        using dmaChComponent = relayAuxDmaChannelComponent;
        using debug = void;
        using tp = tp1;
        using src = crsf_in::input;
        using dest = crsfBuffer;
    };
    struct IBusConfig {
        using pin = sbus_crsf_pin;
        using clock = Devices::clock;
        using systemTimer = Devices::systemTimer;
#ifdef USE_UART_2
        using dmaChComponent = relay1DmaChannelComponent;
#else
        using dmaChRead = relay1DmaChannel;
#endif
        using debug = void;
        using tp = void;
    };
    struct SBusConfig {
        using pin = sbus_crsf_pin;
        using clock = Devices::clock;
        using systemTimer = Devices::systemTimer;
#ifdef USE_UART_2
        using dmaChComponent = relay1DmaChannelComponent;
#else
        using dmaChRead = relay1DmaChannel;
#endif
        using debug = void;
        using tp = void;
    };
    struct Sumdv3Config {
        using pin = sbus_crsf_pin;
        using clock = Devices::clock;
        using systemTimer = Devices::systemTimer;
#ifdef USE_UART_2
        using dmaChComponent = relay1DmaChannelComponent;
#else
        using dmaChRead = relay1DmaChannel;
#endif
        using debug = Devices::debug;
        using tp = void;
    };

    struct InputConfig {
        using stream1 = crsf_in::input;
        using stream2 = Config::relays; // relay connector
        using stream3 = void; // aux
    };

    struct PulseConfig {
        using pin = pulse_pin;
        using clock = Devices::clock;
        using timer = systemTimer;
        using dmaCh = pulseInDmaChannel;
        using debug = Devices::debug;
        using tp = void;
    };

    using polars = Meta::List<polar1, polar2>;

    using telem = Telemetry<crsfBuffer, storage, typename Config::servos, typename Config::escs, debug>;

    using channelCallback = ChannelCallback<polars, typename Config::servos, typename Config::escs, typename Config::relays, typename Config::auxes, telem, storage>;

    struct CrsfCallbackConfig {
        using storage = Config::storage;
        using timer = systemTimer;
        using src = crsf_in;
        using servos = Config::servos;
        using escs = Config::escs;
        using relays = Config::relays;
        using auxes = Config::auxes;
        using mapper = inputs;
        using telemetry = telem;
        using polars = Devices::polars;
        using esc32ascii_1 = Devices::esc32ascii_1;
        using esc32ascii_2 = Devices::esc32ascii_2;
        using tp = void;
    };

    struct CrsfConfig {
        using rxpin = crsfrx;
        using txpin = crsftx;
        using systemTimer = Devices::systemTimer;
        using clock = Devices::clock;
        using dmaChRead  = csrfInDmaChannelComponent1;
        using dmaChWrite = csrfInDmaChannelComponent2;
        using debug = Devices::debug;
        using tp = tp1;
        using callback = CrsfCallback<CrsfCallbackConfig, debug>;
        static inline constexpr uint8_t fifoSize = 16;
    };


    struct SBus1Config {
        using clock = Devices::clock;
        using debug = void;
#ifdef USE_UART_2
        using dmaChComponent = sbus1DmaChannelComponent;
#else
        using dmaChRW = sbus1DmaChannel;
#endif
        using systemTimer = Devices::systemTimer;
        using adapter = crsf_in::input;
        using pin = sbus_crsf_pin;
        using tp = void;
    };
    struct WS1Config {
        using pin = srv1_pin;
        using polar = polar1;
#ifdef USE_UART_2
        using dmaChComponent = srv1DmaChannelComponent;
#else
        using dmaChRW = srv1DmaChannel;
#endif
        using timer = systemTimer;
        using clk = clock;
        using tp = void;
        using dbg = void;
        using storage = Devices::storage;
    };
    struct WS2Config {
        using pin = srv2_pin;
        using polar = polar2;
#ifdef USE_UART_2
        using dmaChComponent = srv2DmaChannelComponent;
#else
        using dmaChRW = srv2DmaChannel;
#endif
        using timer = systemTimer;
        using clk = clock;
        using tp = void;
        using dbg = void;
        using storage = Devices::storage;
    };
    struct SerialConfig1 {
        using clock = Devices::clock;
        using systemTimer = Devices::systemTimer;
        using dmaChRW = esc1DmaChannel;
        using dmaChComponent = esc1DmaChannelComponent;
        using pin = esc1_pin;
        using debug = void;
        using tp = void;
    };
    struct SerialConfig2 {
        using clock = Devices::clock;
        using systemTimer = Devices::systemTimer;
        using dmaChRW = esc2DmaChannel;
        using dmaChComponent = esc2DmaChannelComponent;
        using pin = esc2_pin_1;
        using debug = void;
        using tp = void;
    };

    struct VEscConfig1 {
        using clock = Devices::clock;
        using systemTimer = Devices::systemTimer;
        using dmaChComponent = esc1DmaChannelComponent;
        using pin = esc1_pin;
        using debug = void;
        using tp = void;
    };
    struct VEscConfig2 {
        using clock = Devices::clock;
        using systemTimer = Devices::systemTimer;
        using dmaChComponent = esc2DmaChannelComponent;
        using pin = esc2_pin_1;
        using debug = void;
        using tp = void;
    };
    struct RelayDebug {
        using debug = void;
        using tp = void;
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
#ifdef USE_UART_2
        debug::init();
#else
        debug::init();
        debug::baud(115200);
        debug::rxtxswap(true);
        debugrx::afunction(6);
        debugrx::template pullup<true>();
#endif
#endif

        crsf_in::init();

        sda3::openDrain();
        scl3::openDrain();
        sda3::afunction(6);
        scl3::afunction(6);
        i2c::init();

        // adc::init();
        // adc::oversample(8); // 256

        tp1::template dir<Mcu::Output>();
        tp3::template dir<Mcu::Output>();
    }
};


