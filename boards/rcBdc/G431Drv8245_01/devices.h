#pragma once

#include <cstring>

#include "etl/fixedvector.h"
#include "etl/stackstring.h"

#include "meta.h"

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "pwm.h"
#include "usart.h"
#include "si5351.h"
#include "i2c.h"
#include "timer.h"
#include "opamp.h"
#include "adc.h"
#include "dac.h"
#include "clock.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "gpio.h"
#include "tick.h"
#include "meta.h"
#include "rc/rc.h"
#include "rc/crsf.h"
#include "rc/sport.h"
#include "rc/roboremo.h"
#include "rc/escape.h"
#include "rc/sbus2.h"
#include "rc/vesc.h"
#include "bluetooth/hc05.h"
#include "motor/bdc.h"
#include "blinker.h"
#include "comparator.h"

#include "crsf.h"
#include "telemetry.h"

struct ESC01; // nur pwm
struct ESC02; // mit adc trigger über timer3 (geht nicht direkt)
struct ESC03; // mit adc trigger über timer3 isr
struct ESC04; // mit adc trigger über timer3 isr und dma, ibus
struct ESC05; // mit adc trigger über timer3, dma, ibus, variable pwm Frequenz

struct ESC10; // neue HW (4lagig)
struct ESC11;
struct ESC12;
struct ESC13;
struct ESC14;
struct ESC15;

struct ESC20; // CRSF

using namespace Mcu::Stm;
using namespace std::literals::chrono_literals;

template<typename HW, typename Config, typename MCU = void>
struct Devices;

#if defined (USE_DEVICES2)

template<typename Config>
struct Devices<ESC20, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>; // besser wegen Abwärme im BD433 (bis 24V)
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
#ifdef USE_GNUPLOT
    using trace = Arm::Trace<clock, 10_MHz, 4096>;
#else
    using trace = Arm::Trace<clock, 10_MHz, 1024>;
#endif

    using store = Config::storage;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    static inline constexpr float adc2Voltage(const auto a) {
        constexpr float r1 = 91'000;
        constexpr float r2 = 10'000;
        constexpr float cal = 0.94;
        constexpr float vref = 3.3f;
        constexpr float max = 4095.0f;

        return cal * (a * vref) * (r1 + r2) / (max * r2);
    }
    static inline constexpr float adc2Current(const auto i) {
        constexpr float iFactor = 6150;
        // constexpr float ri = 1'000;
        constexpr float ri = 8'200;
        constexpr float vref = 3.3f;
        constexpr float max = 4095.0f;

        return (i * vref * iFactor) / (max * ri);
    }

    // PA9 Bus : USART1-TX, AF7

    struct CrsfAdapterConfig;
    using servo_pa = RC::Protokoll::Crsf::V2::Adapter<0, CrsfAdapterConfig, MCU>;
    using servo = Mcu::Stm::Uart<1, servo_pa, 0, char, clock, MCU>;
    using servotx = Mcu::Stm::Pin<gpioa, 9, MCU>;

    using sensor_uart = Mcu::Stm::Uart<2, void, 128, std::byte, clock, MCU>;
    using sensorrxtx = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using crsf_out= RC::Protokoll::Crsf::V2::Generator<sensor_uart, systemTimer, MCU>;

    struct CrsfCallbackConfig {
        using telem_out = crsf_out;
        using timer = systemTimer;
        using storage = store;
        using notifier = Config::notifier;
        using speed = Config::speed;
    };

    using crsfCallback = CrsfCallback<CrsfCallbackConfig, trace>;

    struct CrsfAdapterConfig {
        using out = crsf_out;
        using dbg = void;
        using callback = crsfCallback;
        using timer = systemTimer;
    };

    // PA2 Telem : USART2-TX, AF7
    // ggf. auch High-Spee Werte Ausgabe (nach Unterabtastung)
    // template<typename PA>
    // using ibus_uart = Mcu::Stm::Uart<2, PA, 16, std::byte, clock, MCU>;
    // using ibus_sensor = RC::Protokoll::IBus::Sensor<ibus_uart, systemTimer, Meta::List<RpmProvider, CurrentProvider, VoltageProvider>, void, true, true>;
    // using ibusrxtx = Mcu::Stm::Pin<gpioa, 2, MCU>;


           // PB5 Led
    using led = Mcu::Stm::Pin<gpiob, 5, MCU>;
    using ledBlinker = External::Blinker<led, systemTimer>;

           // PB4 Fault
           // PA15 nSleep : TIM2-CH1
           // PA7 Pwm1 : TIM3-CH1, AF2
           // PA6 Pwm2 : TIM3-CH2, AF2
           //            TIM3-CH3 as trgo
           // PB0 Disable

    using in1 = Mcu::Stm::Pin<gpioa, 6, MCU>; // CC1
    using in2 = Mcu::Stm::Pin<gpioa, 7, MCU>; // CC2
    using pwm = Mcu::Stm::Motor::Identification::BdcWithPins<3, 4, in1, in2, clock, MCU>;

    using disable = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using nsleep = Mcu::Stm::Pin<gpioa, 15, MCU>;
    using fault = Mcu::Stm::Pin<gpiob, 4, MCU>;
    using nsleepPulseWaiter = Mcu::Stm::Waiter<6, std::integral_constant<uint16_t, (35 * clock::config::f.value)>,
                                                  std::integral_constant<uint16_t, 0>, MCU>; // 35µs

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

    // besser auch die PGA genutzt
    // PA0 VInSense : ADC12-IN1
    // PA3 ISense : ADC1-IN13 (Opamp1)
    // Temp: ADC1-IN16

    using adcDmaChannel = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using adcDmaStorage = std::array<volatile uint16_t, 3>;
    using adc = Mcu::Stm::V3::Adc<1, Meta::NList<13, 1, 16>, pwm, adcDmaChannel, adcDmaStorage, Meta::List<EndOfSequence>, MCU>; // opamp1
    using isense = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using vsense = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pga = Mcu::Stm::PGA<1, MCU>;

    // PA1 Comparator Lmt01
    using compinp = Mcu::Stm::Pin<gpioa, 1, MCU>;

    // PA4 Testpunkt0, DAC1-Out1
    // PA5 Testpunkt1, DAC1-Out2
    // PB6 Testpunkt2

    using tp0 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using tp1 = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using tp2 = Mcu::Stm::Pin<gpiob, 6, MCU>;
    // using dac = Mcu::Stm::Dac<1, MCU>;
    using comp1 = Mcu::Stm::Comparator<1, compinp, Mcu::Stm::VRefDiv<3, 4>, tp2>;

    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        gpioa::init();
        gpiob::init();
        gpioc::init();
        gpiof::init();


               // sbus::init();
               // sbus_uart::init();
               // sbus_uart::baud(100'000);
               // sbus_uart::parity(true);
               // sbus_uart::invert(true);
               // // sbus_uart::template halfDuplex<true>();
               // // sbus_uart::rxtxswap(true);
               // sbustx::afunction(7);
               // sbustx::template pulldown<true>();

        led::template dir<Mcu::Output>();

        tp0::template dir<Mcu::Output>();
        tp1::template dir<Mcu::Output>();
        tp2::template dir<Mcu::Output>();
        // dac::init();

        isense::analog();
        vsense::analog();
        pga::init();
        pga::input(1);
        pga::gain(0);
        adc::init();

        pwm::init();
        pwm::duty(0);
        fault::pullup();
        fault::template dir<Mcu::Input>();
        // fault::template dir<Mcu::Output>();

        disable::template dir<Mcu::Output>();
        disable::reset();
        in1::template dir<Mcu::Output>();
        in2::template dir<Mcu::Output>();
        // in1::afunction(2);
        // in2::afunction(2);
        nsleep::template dir<Mcu::Output>();
        nsleep::set();
        nsleepPulseWaiter::init();

        servotx::afunction(7);
        servotx::pullup();
        servo::init();
        servo::baud(420'000);
        servo::rxtxswap(true);

        // ibusrxtx::afunction(7);
        // ibusrxtx::pullup();
        // ibusrxtx::openDrain();
        // ibus_sensor::init();

        sensorrxtx::afunction(7);
        // sensorrxtx::pullup();
        // sensorrxtx::openDrain();
        sensor_uart::init();
        sensor_uart::baud(420'000);

        // hstx::afunction(7);
        // hsout::init();
        // // hsout::baud(1'152'000);
        // hsout::baud(921'600);
        // // hsout::baud(115'200);

        comp1::init();
        comp1::enableInt();

        tp2::set();
        tp2::reset();
    }
};

#else

template<typename X>
struct Devices<ESC15, X, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>; // besser wegen Abwärme im BD433 (bis 24V)
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 10_MHz, 1024>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    static inline constexpr float adc2Voltage(const auto a) {
        constexpr float r1 = 91'000;
        constexpr float r2 = 10'000;
        constexpr float cal = 0.96;
        constexpr float vref = 3.3f;
        constexpr float max = 4095.0f;

        return cal * (a * vref) * (r1 + r2) / (max * r2);
    }
    static inline constexpr float adc2Current(const auto i) {
        constexpr float iFactor = 6150;
        constexpr float ri = 1'000;
        constexpr float vref = 3.3f;
        constexpr float max = 4095.0f;

        return (i * vref * iFactor) / (max * ri);
    }

    // PA9 Bus : USART1-TX, AF7

    using servo_pa = RC::Protokoll::IBus::Adapter<0, void>;
    using servo = Mcu::Stm::Uart<1, servo_pa, 0, char, clock, MCU>;
    using servotx = Mcu::Stm::Pin<gpioa, 9, MCU>;

    // PA2 Telem : USART2-TX, AF7
    // ggf. auch High-Spee Werte Ausgabe (nach Unterabtastung)
    // template<typename PA>
    // using ibus_uart = Mcu::Stm::Uart<2, PA, 16, std::byte, clock, MCU>;
    // using ibus_sensor = RC::Protokoll::IBus::Sensor<ibus_uart, systemTimer, Meta::List<RpmProvider, CurrentProvider, VoltageProvider>, void, true, true>;
    // using ibusrxtx = Mcu::Stm::Pin<gpioa, 2, MCU>;

    using hsout = Mcu::Stm::Uart<2, void, 4096, char, clock, MCU>;
    using hstx = Mcu::Stm::Pin<gpioa, 2, MCU>;

           // PB5 Led
    using led = Mcu::Stm::Pin<gpiob, 5, MCU>;

           // PB4 Fault
           // PA15 nSleep : TIM2-CH1
           // PA7 Pwm1 : TIM3-CH1, AF2
           // PA6 Pwm2 : TIM3-CH2, AF2
           //            TIM3-CH3 as trgo
           // PB0 Disable

    // using pwm = Mcu::Stm::Motor::Bdc<3, clock, MCU>;

    using pwm = Mcu::Stm::Motor::Identification::Bdc<3, 4, clock, MCU>;

    using in1 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using in2 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using disable = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using nsleep = Mcu::Stm::Pin<gpioa, 15, MCU>;
    using fault = Mcu::Stm::Pin<gpiob, 4, MCU>;
    using nsleepPulseWaiter = Mcu::Stm::Waiter<6, std::integral_constant<uint16_t, (35 * clock::config::f.value)>,
                                                  std::integral_constant<uint16_t, 0>, MCU>; // 35µs

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

   // besser auch die PGA genutzt
   // PA0 VInSense : ADC12-IN1
   // PA3 ISense : ADC1-IN4

    using adcDmaChannel = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using adcDmaStorage = std::array<volatile uint16_t, 2>;
    // using adcDmaStorage = std::array<volatile uint16_t, 2000>; // fuer Messungen max 1000 Samples
    using adc = Mcu::Stm::V3::Adc<1, Meta::NList<13, 1>, pwm, adcDmaChannel, adcDmaStorage, Meta::List<EndOfSequence>, MCU>; // opamp1
    using isense = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using vsense = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pga = Mcu::Stm::PGA<1, MCU>;

    // PA1 Comparator Lmt01

    // PA4 Testpunkt0, DAC1-Out1
    // PA5 Testpunkt1, DAC1-Out2
    // PB6 Testpunkt2

    using tp0 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using tp1 = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using tp2 = Mcu::Stm::Pin<gpiob, 6, MCU>;
    // using dac = Mcu::Stm::Dac<1, MCU>;

    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        gpioa::init();
        gpiob::init();
        gpioc::init();
        gpiof::init();


               // sbus::init();
               // sbus_uart::init();
               // sbus_uart::baud(100'000);
               // sbus_uart::parity(true);
               // sbus_uart::invert(true);
               // // sbus_uart::template halfDuplex<true>();
               // // sbus_uart::rxtxswap(true);
               // sbustx::afunction(7);
               // sbustx::template pulldown<true>();

        led::template dir<Mcu::Output>();

        tp0::template dir<Mcu::Output>();
        tp1::template dir<Mcu::Output>();
        tp2::template dir<Mcu::Output>();
        // dac::init();

        isense::analog();
        vsense::analog();
        pga::init();
        pga::input(1);
        pga::gain(0);
        adc::init();

        pwm::init();
        fault::pullup();
        fault::template dir<Mcu::Input>();
        // fault::template dir<Mcu::Output>();

        disable::template dir<Mcu::Output>();
        disable::reset();
        in1::template dir<Mcu::Output>();
        in2::template dir<Mcu::Output>();
        // in1::afunction(2);
        // in2::afunction(2);
        nsleep::template dir<Mcu::Output>();
        nsleep::set();
        nsleepPulseWaiter::init();

        servotx::afunction(7);
        servotx::pullup();
        servo::init();
        servo::baud(115'200);
        servo::rxtxswap(true);

        // ibusrxtx::afunction(7);
        // ibusrxtx::pullup();
        // ibusrxtx::openDrain();
        // ibus_sensor::init();

        hstx::afunction(7);
        hsout::init();
        // hsout::baud(1'152'000);
        hsout::baud(921'600);
        // hsout::baud(115'200);

        tp2::set();
        tp2::reset();
    }
};

template<typename X>
struct Devices<ESC14, X, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>; // besser wegen Abwärme im BD433 (bis 24V)
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 10_MHz, 1024>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    static inline constexpr float adc2Voltage(const auto a) {
        constexpr float r1 = 91'000;
        constexpr float r2 = 10'000;
        constexpr float cal = 0.96;
        constexpr float vref = 3.3f;
        constexpr float max = 4095.0f;

        return cal * (a * vref) * (r1 + r2) / (max * r2);
    }
    static inline constexpr float adc2Current(const auto i) {
        constexpr float iFactor = 6150;
        constexpr float ri = 1'000;
        constexpr float vref = 3.3f;
        constexpr float max = 4095.0f;

        return (i * vref * iFactor) / (max * ri);
    }

    // PA9 Bus : USART1-TX, AF7

    using servo_pa = RC::Protokoll::IBus::Adapter<0, void>;
    using servo = Mcu::Stm::Uart<1, servo_pa, 0, char, clock, MCU>;
    using servotx = Mcu::Stm::Pin<gpioa, 9, MCU>;

    // PA2 Telem : USART2-TX, AF7
    // ggf. auch High-Spee Werte Ausgabe (nach Unterabtastung)
    // template<typename PA>
    // using ibus_uart = Mcu::Stm::Uart<2, PA, 16, std::byte, clock, MCU>;
    // using ibus_sensor = RC::Protokoll::IBus::Sensor<ibus_uart, systemTimer, Meta::List<RpmProvider, CurrentProvider, VoltageProvider>, void, true, true>;
    // using ibusrxtx = Mcu::Stm::Pin<gpioa, 2, MCU>;

    using hsout = Mcu::Stm::Uart<2, void, 4096, char, clock, MCU>;
    using hstx = Mcu::Stm::Pin<gpioa, 2, MCU>;

           // PB5 Led
    using led = Mcu::Stm::Pin<gpiob, 5, MCU>;

           // PB4 Fault
           // PA15 nSleep : TIM2-CH1
           // PA7 Pwm1 : TIM3-CH1, AF2
           // PA6 Pwm2 : TIM3-CH2, AF2
           //            TIM3-CH3 as trgo
           // PB0 Disable

    // using pwm = Mcu::Stm::Motor::Bdc<3, clock, MCU>;

    using pwm = Mcu::Stm::Motor::Identification::Bdc<3, 4, clock, MCU>;

    using in1 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using in2 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using disable = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using nsleep = Mcu::Stm::Pin<gpioa, 15, MCU>;
    using fault = Mcu::Stm::Pin<gpiob, 4, MCU>;
    using nsleepPulseWaiter = Mcu::Stm::Waiter<6, std::integral_constant<uint16_t, (35 * clock::config::f.value)>,
                                                  std::integral_constant<uint16_t, 0>, MCU>; // 35µs

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

   // besser auch die PGA genutzt
   // PA0 VInSense : ADC12-IN1
   // PA3 ISense : ADC1-IN4

    using adcDmaChannel = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using adcDmaStorage = std::array<volatile uint16_t, 2>;
    // using adcDmaStorage = std::array<volatile uint16_t, 2000>; // fuer Messungen max 1000 Samples
    using adc = Mcu::Stm::V3::Adc<1, Meta::NList<13, 1>, pwm, adcDmaChannel, adcDmaStorage, Meta::List<EndOfSequence>, MCU>; // opamp1
    using isense = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using vsense = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pga = Mcu::Stm::PGA<1, MCU>;

    // PA1 Comparator Lmt01

    // PA4 Testpunkt0, DAC1-Out1
    // PA5 Testpunkt1, DAC1-Out2
    // PB6 Testpunkt2

    using tp0 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using tp1 = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using tp2 = Mcu::Stm::Pin<gpiob, 6, MCU>;
    // using dac = Mcu::Stm::Dac<1, MCU>;

    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        gpioa::init();
        gpiob::init();
        gpioc::init();
        gpiof::init();


               // sbus::init();
               // sbus_uart::init();
               // sbus_uart::baud(100'000);
               // sbus_uart::parity(true);
               // sbus_uart::invert(true);
               // // sbus_uart::template halfDuplex<true>();
               // // sbus_uart::rxtxswap(true);
               // sbustx::afunction(7);
               // sbustx::template pulldown<true>();

        led::template dir<Mcu::Output>();

        tp0::template dir<Mcu::Output>();
        tp1::template dir<Mcu::Output>();
        tp2::template dir<Mcu::Output>();
        // dac::init();

        isense::analog();
        vsense::analog();
        pga::init();
        pga::input(1);
        pga::gain(0);
        adc::init();

        pwm::init();
        fault::pullup();
        fault::template dir<Mcu::Input>();
        // fault::template dir<Mcu::Output>();

        disable::template dir<Mcu::Output>();
        disable::reset();
        in1::template dir<Mcu::Output>();
        in2::template dir<Mcu::Output>();
        // in1::afunction(2);
        // in2::afunction(2);
        nsleep::template dir<Mcu::Output>();
        nsleep::set();
        nsleepPulseWaiter::init();

        servotx::afunction(7);
        servotx::pullup();
        servo::init();
        servo::baud(115'200);
        servo::rxtxswap(true);

        // ibusrxtx::afunction(7);
        // ibusrxtx::pullup();
        // ibusrxtx::openDrain();
        // ibus_sensor::init();

        hstx::afunction(7);
        hsout::init();
        // hsout::baud(1'152'000);
        hsout::baud(921'600);
        // hsout::baud(115'200);

        tp2::set();
        tp2::reset();
    }
};

template<typename X>
struct Devices<ESC13, X, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>; // besser wegen Abwärme im BD433 (bis 24V)
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 2_MHz, 128>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    // PA9 Bus : USART1-TX, AF7

    using servo_pa = RC::Protokoll::IBus::Adapter<0, void>;
    using servo = Mcu::Stm::Uart<1, servo_pa, 0, char, clock, MCU>;
    using servotx = Mcu::Stm::Pin<gpioa, 9, MCU>;

    // PA2 Telem : USART2-TX, AF7
    // ggf. auch High-Spee Werte Ausgabe (nach Unterabtastung)
    // template<typename PA>
    // using ibus_uart = Mcu::Stm::Uart<2, PA, 16, std::byte, clock, MCU>;
    // using ibus_sensor = RC::Protokoll::IBus::Sensor<ibus_uart, systemTimer, Meta::List<RpmProvider, CurrentProvider, VoltageProvider>, void, true, true>;
    // using ibusrxtx = Mcu::Stm::Pin<gpioa, 2, MCU>;

    using hsout = Mcu::Stm::Uart<2, void, 4096, char, clock, MCU>;
    using hstx = Mcu::Stm::Pin<gpioa, 2, MCU>;

           // PB5 Led
    using led = Mcu::Stm::Pin<gpiob, 5, MCU>;

           // PB4 Fault
           // PA15 nSleep : TIM2-CH1
           // PA7 Pwm1 : TIM3-CH1, AF2
           // PA6 Pwm2 : TIM3-CH2, AF2
           //            TIM3-CH3 as trgo
           // PB0 Disable

    // using pwm = Mcu::Stm::Motor::Bdc<3, clock, MCU>;

    using pwm = Mcu::Stm::Motor::Identification::Bdc<3, 4, clock, MCU>;

    using in1 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using in2 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using disable = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using nsleep = Mcu::Stm::Pin<gpioa, 15, MCU>;
    using fault = Mcu::Stm::Pin<gpiob, 4, MCU>;
    using nsleepPulseWaiter = Mcu::Stm::Waiter<6, std::integral_constant<uint16_t, (35 * clock::config::f.value)>,
                                                  std::integral_constant<uint16_t, 0>, MCU>; // 35µs

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

   // besser auch die PGA genutzt
   // PA0 VInSense : ADC12-IN1
   // PA3 ISense : ADC1-IN4

    using adcDmaChannel = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    // using adcDmaStorage = std::array<volatile uint16_t, 2>;
    using adcDmaStorage = std::array<volatile uint16_t, 2000>; // fuer Messungen max 1000 Samples
    using adc = Mcu::Stm::V3::Adc<1, Meta::NList<13, 1>, pwm, adcDmaChannel, adcDmaStorage, Meta::List<EndOfSequence>, MCU>; // opamp1
    using isense = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using vsense = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pga = Mcu::Stm::PGA<1, MCU>;

    // PA1 Comparator Lmt01

    // PA4 Testpunkt0, DAC1-Out1
    // PA5 Testpunkt1, DAC1-Out2
    // PB6 Testpunkt2

    using tp0 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using tp1 = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using tp2 = Mcu::Stm::Pin<gpiob, 6, MCU>;
    // using dac = Mcu::Stm::Dac<1, MCU>;

    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        gpioa::init();
        gpiob::init();
        gpioc::init();
        gpiof::init();


               // sbus::init();
               // sbus_uart::init();
               // sbus_uart::baud(100'000);
               // sbus_uart::parity(true);
               // sbus_uart::invert(true);
               // // sbus_uart::template halfDuplex<true>();
               // // sbus_uart::rxtxswap(true);
               // sbustx::afunction(7);
               // sbustx::template pulldown<true>();

        led::template dir<Mcu::Output>();

        tp0::template dir<Mcu::Output>();
        tp1::template dir<Mcu::Output>();
        tp2::template dir<Mcu::Output>();
        // dac::init();

        isense::analog();
        vsense::analog();
        pga::init();
        pga::input(1);
        pga::gain(0);
        adc::init();

        pwm::init();
        fault::pullup();
        fault::template dir<Mcu::Input>();
        // fault::template dir<Mcu::Output>();

        disable::template dir<Mcu::Output>();
        disable::reset();
        in1::template dir<Mcu::Output>();
        in2::template dir<Mcu::Output>();
        // in1::afunction(2);
        // in2::afunction(2);
        nsleep::template dir<Mcu::Output>();
        nsleep::set();
        nsleepPulseWaiter::init();

        servotx::afunction(7);
        servotx::pullup();
        servo::init();
        servo::baud(115'200);
        servo::rxtxswap(true);

        // ibusrxtx::afunction(7);
        // ibusrxtx::pullup();
        // ibusrxtx::openDrain();
        // ibus_sensor::init();

        hstx::afunction(7);
        hsout::init();
        // hsout::baud(1'152'000);
        hsout::baud(921'600);
        // hsout::baud(115'200);

        tp2::set();
        tp2::reset();
    }
};

template<typename X>
struct Devices<ESC12, X, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>; // besser wegen Abwärme im BD433 (bis 24V)
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 2_MHz, 128>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    // PA9 Bus : USART1-TX, AF7

    using servo_pa = RC::Protokoll::IBus::Adapter<0, void>;
    using servo = Mcu::Stm::Uart<1, servo_pa, 0, char, clock, MCU>;
    using servotx = Mcu::Stm::Pin<gpioa, 9, MCU>;

    // PA2 Telem : USART2-TX, AF7
    // ggf. auch High-Spee Werte Ausgabe (nach Unterabtastung)
    // template<typename PA>
    // using ibus_uart = Mcu::Stm::Uart<2, PA, 16, std::byte, clock, MCU>;
    // using ibus_sensor = RC::Protokoll::IBus::Sensor<ibus_uart, systemTimer, Meta::List<RpmProvider, CurrentProvider, VoltageProvider>, void, true, true>;
    // using ibusrxtx = Mcu::Stm::Pin<gpioa, 2, MCU>;

    using hsout = Mcu::Stm::Uart<2, void, 256, char, clock, MCU>;
    using hstx = Mcu::Stm::Pin<gpioa, 2, MCU>;

           // PB5 Led
    using led = Mcu::Stm::Pin<gpiob, 5, MCU>;

           // PB4 Fault
           // PA15 nSleep : TIM2-CH1
           // PA7 Pwm1 : TIM3-CH1, AF2
           // PA6 Pwm2 : TIM3-CH2, AF2
           //            TIM3-CH3 as trgo
           // PB0 Disable

    // using pwm = Mcu::Stm::Motor::Bdc<3, clock, MCU>;

    using pwm = Mcu::Stm::Motor::Identification::Bdc<3, 4, clock, MCU>;

    using in1 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using in2 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using disable = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using nsleep = Mcu::Stm::Pin<gpioa, 15, MCU>;
    using fault = Mcu::Stm::Pin<gpiob, 4, MCU>;
    using nsleepPulseWaiter = Mcu::Stm::Waiter<6, std::integral_constant<uint16_t, (35 * clock::config::f.value)>,
                                                  std::integral_constant<uint16_t, 0>, MCU>; // 35µs

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

   // besser auch die PGA genutzt
   // PA0 VInSense : ADC12-IN1
   // PA3 ISense : ADC1-IN4

    using adcDmaChannel = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using adcDmaStorage = std::array<volatile uint16_t, 2>;
    using adc = Mcu::Stm::V3::Adc<1, Meta::NList<13, 1>, pwm, adcDmaChannel, adcDmaStorage, Meta::List<EndOfSequence>, MCU>; // opamp1
    using isense = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using vsense = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pga = Mcu::Stm::PGA<1, MCU>;

    // PA1 Comparator Lmt01

    // PA4 Testpunkt0, DAC1-Out1
    // PA5 Testpunkt1, DAC1-Out2
    // PB6 Testpunkt2

    using tp0 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using tp1 = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using tp2 = Mcu::Stm::Pin<gpiob, 6, MCU>;
    // using dac = Mcu::Stm::Dac<1, MCU>;

    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        gpioa::init();
        gpiob::init();
        gpioc::init();
        gpiof::init();


               // sbus::init();
               // sbus_uart::init();
               // sbus_uart::baud(100'000);
               // sbus_uart::parity(true);
               // sbus_uart::invert(true);
               // // sbus_uart::template halfDuplex<true>();
               // // sbus_uart::rxtxswap(true);
               // sbustx::afunction(7);
               // sbustx::template pulldown<true>();

        led::template dir<Mcu::Output>();

        tp0::template dir<Mcu::Output>();
        tp1::template dir<Mcu::Output>();
        tp2::template dir<Mcu::Output>();
        // dac::init();

        isense::analog();
        vsense::analog();
        pga::init();
        pga::input(1);
        pga::gain(0);
        adc::init();

        pwm::init();
        fault::pullup();
        fault::template dir<Mcu::Input>();
        // fault::template dir<Mcu::Output>();

        disable::template dir<Mcu::Output>();
        disable::reset();
        in1::template dir<Mcu::Output>();
        in2::template dir<Mcu::Output>();
        // in1::afunction(2);
        // in2::afunction(2);
        nsleep::template dir<Mcu::Output>();
        nsleep::set();
        nsleepPulseWaiter::init();

        servotx::afunction(7);
        servotx::pullup();
        servo::init();
        servo::baud(115'200);
        servo::rxtxswap(true);

        // ibusrxtx::afunction(7);
        // ibusrxtx::pullup();
        // ibusrxtx::openDrain();
        // ibus_sensor::init();

        hstx::afunction(7);
        hsout::init();
        hsout::baud(921'600);
        // hsout::baud(115'200);

        tp2::set();
        tp2::reset();
    }
};

template<typename X>
struct Devices<ESC11, X, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>; // besser wegen Abwärme im BD433 (bis 24V)
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 2_MHz, 128>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    // PA9 Bus : USART1-TX, AF7

    using servo_pa = RC::Protokoll::IBus::Adapter<0, void>;
    using servo = Mcu::Stm::Uart<1, servo_pa, 0, char, clock, MCU>;
    using servotx = Mcu::Stm::Pin<gpioa, 9, MCU>;

    // PA2 Telem : USART2-TX, AF7
    // ggf. auch High-Spee Werte Ausgabe (nach Unterabtastung)
    // template<typename PA>
    // using ibus_uart = Mcu::Stm::Uart<2, PA, 16, std::byte, clock, MCU>;
    // using ibus_sensor = RC::Protokoll::IBus::Sensor<ibus_uart, systemTimer, Meta::List<RpmProvider, CurrentProvider, VoltageProvider>, void, true, true>;
    // using ibusrxtx = Mcu::Stm::Pin<gpioa, 2, MCU>;

    using hsout = Mcu::Stm::Uart<2, void, 256, char, clock, MCU>;
    using hstx = Mcu::Stm::Pin<gpioa, 2, MCU>;

           // PB5 Led
    using led = Mcu::Stm::Pin<gpiob, 5, MCU>;

           // PB4 Fault
           // PA15 nSleep : TIM2-CH1
           // PA7 Pwm1 : TIM3-CH1, AF2
           // PA6 Pwm2 : TIM3-CH2, AF2
           //            TIM3-CH3 as trgo
           // PB0 Disable

    using pwm = Mcu::Stm::Motor::Bdc<3, clock, MCU>;
    using in1 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using in2 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using disable = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using nsleep = Mcu::Stm::Pin<gpioa, 15, MCU>;
    using fault = Mcu::Stm::Pin<gpiob, 4, MCU>;
    using nsleepPulseWaiter = Mcu::Stm::Waiter<6, std::integral_constant<uint16_t, (35 * clock::config::f.value)>,
                                                  std::integral_constant<uint16_t, 0>, MCU>; // 35µs

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

   // besser auch die PGA genutzt
   // PA0 VInSense : ADC12-IN1
   // PA3 ISense : ADC1-IN4

    using adcDmaChannel = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using adcDmaStorage = std::array<volatile uint16_t, 2>;
    using adc = Mcu::Stm::V3::Adc<1, Meta::NList<13, 1>, pwm, adcDmaChannel, adcDmaStorage, Meta::List<EndOfSequence>, MCU>; // opamp1
    using isense = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using vsense = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pga = Mcu::Stm::PGA<1, MCU>;

    // PA1 Comparator Lmt01

    // PA4 Testpunkt0, DAC1-Out1
    // PA5 Testpunkt1, DAC1-Out2
    // PB6 Testpunkt2

    using tp0 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using tp1 = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using tp2 = Mcu::Stm::Pin<gpiob, 6, MCU>;
    // using dac = Mcu::Stm::Dac<1, MCU>;

    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        gpioa::init();
        gpiob::init();
        gpioc::init();
        gpiof::init();


               // sbus::init();
               // sbus_uart::init();
               // sbus_uart::baud(100'000);
               // sbus_uart::parity(true);
               // sbus_uart::invert(true);
               // // sbus_uart::template halfDuplex<true>();
               // // sbus_uart::rxtxswap(true);
               // sbustx::afunction(7);
               // sbustx::template pulldown<true>();

        led::template dir<Mcu::Output>();

        tp0::template dir<Mcu::Output>();
        tp1::template dir<Mcu::Output>();
        tp2::template dir<Mcu::Output>();
        // dac::init();

        isense::analog();
        vsense::analog();
        pga::init();
        pga::input(1);
        pga::gain(0);
        adc::init();

        pwm::init();
        fault::pullup();
        fault::template dir<Mcu::Input>();
        // fault::template dir<Mcu::Output>();

        disable::template dir<Mcu::Output>();
        disable::reset();
        in1::template dir<Mcu::Output>();
        in2::template dir<Mcu::Output>();
        // in1::afunction(2);
        // in2::afunction(2);
        nsleep::template dir<Mcu::Output>();
        nsleep::set();
        nsleepPulseWaiter::init();

        servotx::afunction(7);
        servotx::pullup();
        servo::init();
        servo::baud(115'200);
        servo::rxtxswap(true);

        // ibusrxtx::afunction(7);
        // ibusrxtx::pullup();
        // ibusrxtx::openDrain();
        // ibus_sensor::init();

        hstx::afunction(7);
        hsout::init();
        hsout::baud(921'600);
        // hsout::baud(115'200);

        tp2::set();
        tp2::reset();
    }
};

template<typename Config>
struct Devices<ESC10, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>; // besser wegen Abwärme im BD433 (bis 24V)
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 2_MHz, 128>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    // PA9 Bus : USART1-TX, AF7

    using servo_pa = RC::Protokoll::IBus::Adapter<0, void>;
    using servo = Mcu::Stm::Uart<1, servo_pa, 0, char, clock, MCU>;
    using servotx = Mcu::Stm::Pin<gpioa, 9, MCU>;

    // PA2 Telem : USART2-TX, AF7
    // ggf. auch High-Spee Werte Ausgabe (nach Unterabtastung)
    template<typename PA>
    using ibus_uart = Mcu::Stm::Uart<2, PA, 16, std::byte, clock, MCU>;
    using ibus_sensor = RC::Protokoll::IBus::Sensor<ibus_uart, systemTimer, Meta::List<RpmProvider, CurrentProvider, VoltageProvider>, void, true, true>;
    using ibusrxtx = Mcu::Stm::Pin<gpioa, 2, MCU>;

           // PB5 Led
    using led = Mcu::Stm::Pin<gpiob, 5, MCU>;

           // PB4 Fault
           // PA15 nSleep : TIM2-CH1
           // PA7 Pwm1 : TIM3-CH1, AF2
           // PA6 Pwm2 : TIM3-CH2, AF2
           //            TIM3-CH3 as trgo
           // PB0 Disable

    using pwm = Mcu::Stm::Motor::Bdc<3, clock, MCU>;
    using in1 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using in2 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using disable = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using nsleep = Mcu::Stm::Pin<gpioa, 15, MCU>;
    using fault = Mcu::Stm::Pin<gpiob, 4, MCU>;
    using nsleepPulseWaiter = Mcu::Stm::Waiter<6, std::integral_constant<uint16_t, (35 * clock::config::f.value)>,
                                                  std::integral_constant<uint16_t, 0>, MCU>; // 35µs

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

   // besser auch die PGA genutzt
   // PA0 VInSense : ADC12-IN1
   // PA3 ISense : ADC1-IN4

    using adcDmaChannel = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using adcDmaStorage = std::array<volatile uint16_t, 2>;
    using adc = Mcu::Stm::V3::Adc<1, Meta::NList<13, 1>, pwm, adcDmaChannel, adcDmaStorage, void, MCU>; // opamp1
    using isense = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using vsense = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pga = Mcu::Stm::PGA<1, MCU>;

    // PA1 Comparator Lmt01

    // PA4 Testpunkt0, DAC1-Out1
    // PA5 Testpunkt1, DAC1-Out2
    // PB6 Testpunkt2

    using tp2 = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using dac = Mcu::Stm::Dac<1, MCU>;

    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        gpioa::init();
        gpiob::init();
        gpioc::init();
        gpiof::init();


               // sbus::init();
               // sbus_uart::init();
               // sbus_uart::baud(100'000);
               // sbus_uart::parity(true);
               // sbus_uart::invert(true);
               // // sbus_uart::template halfDuplex<true>();
               // // sbus_uart::rxtxswap(true);
               // sbustx::afunction(7);
               // sbustx::template pulldown<true>();

        led::template dir<Mcu::Output>();

        // tp::template dir<Mcu::Output>();
        dac::init();

        isense::analog();
        vsense::analog();
        pga::init();
        pga::input(1);
        pga::gain(1); // 4
        adc::init();

        pwm::init();
        fault::pullup();
        // fault::template dir<Mcu::Input>();
        fault::template dir<Mcu::Output>();

        disable::template dir<Mcu::Output>();
        disable::reset();
        in1::template dir<Mcu::Output>();
        in2::template dir<Mcu::Output>();
        // in1::afunction(2);
        // in2::afunction(2);
        nsleep::template dir<Mcu::Output>();
        nsleep::set();
        nsleepPulseWaiter::init();

        servotx::afunction(7);
        servotx::pullup();
        servo::init();
        servo::baud(115'200);
        servo::rxtxswap(true);

        ibusrxtx::afunction(7);
        ibusrxtx::pullup();
        ibusrxtx::openDrain();
        ibus_sensor::init();

        tp2::set();
        tp2::reset();
    }
};

template<typename Config>
struct Devices<ESC05, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<60_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>; // besser wegen Abwärme im BD433 (bis 24V)
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 2_MHz, 128>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

           // PA2 Telem : USART2-TX, AF7

           // PA9 Bus : USART1-TX, AF7

    // struct SbusCallback;
    // using sbus_pa = RC::Protokoll::SBus2::Adapter<0, SbusCallback>;
    // using sbus_uart = Mcu::Stm::Uart<2, sbus_pa, 32, std::byte, clock, MCU>;
    // using sbus = RC::Protokoll::SBus2::Fsm<sbus_uart, systemTimer>;
    // using sbustx = Mcu::Stm::Pin<gpiob, 4, MCU>; // rx, AF 7, need swap rxtx-pins

    using servo_pa = RC::Protokoll::IBus::Adapter<0, void>;
    using servo = Mcu::Stm::Uart<1, servo_pa, 0, char, clock, MCU>;
    using servotx = Mcu::Stm::Pin<gpioa, 9, MCU>;

    template<typename PA>
    using ibus_uart = Mcu::Stm::Uart<2, PA, 16, std::byte, clock, MCU>;
    using ibus_sensor = RC::Protokoll::IBus::Sensor<ibus_uart, systemTimer, Meta::List<RpmProvider, CurrentProvider, VoltageProvider>, void, true, true>;
    using ibusrxtx = Mcu::Stm::Pin<gpioa, 2, MCU>;

           // PB7 Led
    using led = Mcu::Stm::Pin<gpiob, 7, MCU>;

           // PB4 Fault
           // PA15 nSleep : TIM2-CH1
           // PA6 Pwm1 : TIM3-CH1, AF2
           // PA4 Pwm2 : TIM3-CH2, AF2
           //            TIM3-CH3 as trgo
           // PB0 Disable

    using pwm = Mcu::Stm::Motor::Bdc<3, clock, MCU>;
    using in1 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using in2 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using disable = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using nsleep = Mcu::Stm::Pin<gpioa, 15, MCU>;
    using fault = Mcu::Stm::Pin<gpiob, 4, MCU>;
    using nsleepPulseWaiter = Mcu::Stm::Waiter<6, std::integral_constant<uint16_t, (35 * clock::config::f.value)>,
                                                  std::integral_constant<uint16_t, 0>, MCU>; // 35µs

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

   // besser auch die PGA genutzt
   // PA0 ISense : ADC12-IN1 (auf Platine geändert -> VSense)
   // PA3 VInSense : ADC1-IN4 (auf Platine geändert -> ISense)

    using adcDmaChannel = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using adcDmaStorage = std::array<volatile uint16_t, 2>;
    using adc = Mcu::Stm::V3::Adc<1, Meta::NList<13, 1>, pwm, adcDmaChannel, adcDmaStorage, void, MCU>; // opamp1
    using isense = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using vsense = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pga = Mcu::Stm::PGA<1, MCU>;

           // PA1 Comparator Lmt01

    // PA5 Testpunkt, DAC1-Out2

    using tp = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using dac = Mcu::Stm::Dac<1, MCU>;

    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        gpioa::init();
        gpiob::init();
        gpioc::init();
        gpiof::init();


               // sbus::init();
               // sbus_uart::init();
               // sbus_uart::baud(100'000);
               // sbus_uart::parity(true);
               // sbus_uart::invert(true);
               // // sbus_uart::template halfDuplex<true>();
               // // sbus_uart::rxtxswap(true);
               // sbustx::afunction(7);
               // sbustx::template pulldown<true>();

        led::template dir<Mcu::Output>();

        // tp::template dir<Mcu::Output>();
        dac::init();

        isense::analog();
        vsense::analog();
        pga::init();
        pga::input(1);
        pga::gain(1); // 4
        adc::init();

        pwm::init();
        fault::pullup();
        // fault::template dir<Mcu::Input>();
        fault::template dir<Mcu::Output>();

        disable::template dir<Mcu::Output>();
        disable::reset();
        in1::template dir<Mcu::Output>();
        in2::template dir<Mcu::Output>();
        // in1::afunction(2);
        // in2::afunction(2);
        nsleep::template dir<Mcu::Output>();
        nsleep::set();
        nsleepPulseWaiter::init();

        servotx::afunction(7);
        servotx::pullup();
        servo::init();
        servo::baud(115'200);
        servo::rxtxswap(true);

        ibusrxtx::afunction(7);
        ibusrxtx::pullup();
        ibusrxtx::openDrain();
        ibus_sensor::init();

        tp::set();
        tp::reset();
    }
};

template<typename Config>
struct Devices<ESC04, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    // using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<60_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>; // besser wegen Abwärme im BD433 (bis 24V)
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 2_MHz, 128>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

           // PA2 Telem : USART2-TX, AF7

           // PA9 Bus : USART1-TX, AF7

    // struct SbusCallback;
    // using sbus_pa = RC::Protokoll::SBus2::Adapter<0, SbusCallback>;
    // using sbus_uart = Mcu::Stm::Uart<2, sbus_pa, 32, std::byte, clock, MCU>;
    // using sbus = RC::Protokoll::SBus2::Fsm<sbus_uart, systemTimer>;
    // using sbustx = Mcu::Stm::Pin<gpiob, 4, MCU>; // rx, AF 7, need swap rxtx-pins


    using servo_pa = RC::Protokoll::IBus::Adapter<0, void>;
    using servo = Mcu::Stm::Uart<1, servo_pa, 0, char, clock, MCU>;
    using servotx = Mcu::Stm::Pin<gpioa, 9, MCU>;

    template<typename PA>
    using ibus_uart = Mcu::Stm::Uart<2, PA, 16, std::byte, clock, MCU>;
    using ibus_sensor = RC::Protokoll::IBus::Sensor<ibus_uart, systemTimer, Meta::List<RpmProvider, CurrentProvider, VoltageProvider>, void, true, true>;
    using ibusrxtx = Mcu::Stm::Pin<gpioa, 2, MCU>;

           // PB7 Led
    using led = Mcu::Stm::Pin<gpiob, 7, MCU>;

           // PB4 Fault
           // PA15 nSleep : TIM2-CH1
           // PA6 Pwm1 : TIM3-CH1, AF2
           // PA4 Pwm2 : TIM3-CH2, AF2
           //            TIM3-CH3 as trgo
           // PB0 Disable

    using pwm = Mcu::Stm::Motor::Bdc<3, clock, MCU>;
    using in1 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using in2 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using disable = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using nsleep = Mcu::Stm::Pin<gpioa, 15, MCU>;
    using fault = Mcu::Stm::Pin<gpiob, 4, MCU>;
    using nsleepPulseWaiter = Mcu::Stm::Waiter<6, std::integral_constant<uint16_t, (35 * clock::config::f.value)>,
                                                  std::integral_constant<uint16_t, 0>, MCU>;

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

   // besser auch die PGA genutzt
   // PA0 ISense : ADC12-IN1 (auf Platine geändert -> VSense)
   // PA3 VInSense : ADC1-IN4 (auf Platine geändert -> ISense)

    using adcDmaChannel = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using adcDmaStorage = std::array<volatile uint16_t, 2>;
    using adc = Mcu::Stm::V3::Adc<1, Meta::NList<13, 1>, pwm, adcDmaChannel, adcDmaStorage, void, MCU>; // opamp1
    // using adc = Mcu::Stm::V3::Adc<1, Meta::NList<4, 1>, pwm, adcDmaChannel, adcDmaStorage, MCU>; // opamp1
    using isense = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using vsense = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pga = Mcu::Stm::PGA<1, MCU>;

           // PA1 Comparator Lmt01

    // PA5 Testpunkt, DAC1-Out2

    using tp = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using dac = Mcu::Stm::Dac<1, MCU>;

    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        gpioa::init();
        gpiob::init();
        gpioc::init();
        gpiof::init();


               // sbus::init();
               // sbus_uart::init();
               // sbus_uart::baud(100'000);
               // sbus_uart::parity(true);
               // sbus_uart::invert(true);
               // // sbus_uart::template halfDuplex<true>();
               // // sbus_uart::rxtxswap(true);
               // sbustx::afunction(7);
               // sbustx::template pulldown<true>();

        led::template dir<Mcu::Output>();

        // tp::template dir<Mcu::Output>();
        dac::init();

        isense::analog();
        vsense::analog();
        pga::init();
        pga::input(1);
        pga::gain(2); // 4
        adc::init();

        pwm::init();
        fault::pullup();
        // fault::template dir<Mcu::Input>();
        fault::template dir<Mcu::Output>();

        disable::template dir<Mcu::Output>();
        disable::reset();
        in1::template dir<Mcu::Output>();
        in2::template dir<Mcu::Output>();
        // in1::afunction(2);
        // in2::afunction(2);
        nsleep::template dir<Mcu::Output>();
        nsleep::set();
        nsleepPulseWaiter::init();

        servotx::afunction(7);
        servotx::pullup();
        servo::init();
        servo::baud(115'200);
        servo::rxtxswap(true);

        ibusrxtx::afunction(7);
        ibusrxtx::pullup();
        ibusrxtx::openDrain();
        ibus_sensor::init();

        tp::set();
        tp::reset();
    }
};

template<typename Config>
struct Devices<ESC03, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 1'000_Hz, Mcu::Stm::HSI>, MCU>;
                                                                     // using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<60_MHz, 30'000_Hz, Mcu::Stm::HSI>, MCU>;
                                                                     using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 2_MHz, 128>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

           // PA2 Telem : USART2-TX, AF7

           // PA9 Bus : USART1-TX, AF7

    struct SbusCallback;
    using sbus_pa = RC::Protokoll::SBus2::Adapter<0, SbusCallback>;
    using sbus_uart = Mcu::Stm::Uart<2, sbus_pa, 32, std::byte, clock, MCU>;
    using sbus = RC::Protokoll::SBus2::Fsm<sbus_uart, systemTimer>;
    using sbustx = Mcu::Stm::Pin<gpiob, 4, MCU>; // rx, AF 7, need swap rxtx-pins


           // PB7 Led
    using led = Mcu::Stm::Pin<gpiob, 7, MCU>;

           // PB4 Fault
           // PA15 nSleep : TIM2-CH1
           // PA6 Pwm1 : TIM3-CH1, AF2
           // PA4 Pwm2 : TIM3-CH2, AF2
           //            TIM3-CH3 as trgo
           // PB0 Disable

    using pwm = Mcu::Stm::Motor::Bdc<3, clock, MCU>;
    using in1 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using in2 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using disable = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using nsleep = Mcu::Stm::Pin<gpioa, 15, MCU>;
    using fault = Mcu::Stm::Pin<gpiob, 4, MCU>;
    using nsleepPulseWaiter = Mcu::Stm::Waiter<6,
                                               std::integral_constant<uint16_t, 35 * 170>,
                                               std::integral_constant<uint16_t, 0>, MCU>;

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

   // besser auch die PGA genutzt
   // PA0 ISense : ADC12-IN1 (auf Platine geändert -> VSense)
   // PA3 VInSense : ADC1-IN4 (auf Platine geändert -> ISense)
    using adc = Mcu::Stm::V2::Adc<1, 13, Mcu::Stm::NoTriggerSource, Mcu::Stm::UseDma<false>, MCU>; // opamp1
    // using adc = Mcu::Stm::Adc<1, 4, Mcu::Stm::NoTriggerSource, Mcu::Stm::UseDma<false>, MCU>;
    // using adc = Mcu::Stm::Adc<1, 4, pwm, Mcu::Stm::UseDma<false>, MCU>;
    using isense = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using vsense = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pga = Mcu::Stm::PGA<1, MCU>;
    using adcDmaChannel1 = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;

           // PA1 Comparator Lmt01

           // PA5 Testpunkt, DAC1-Out2

    using tp = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using dac = Mcu::Stm::Dac<1, MCU>;

    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        gpioa::init();
        gpiob::init();
        gpioc::init();
        gpiof::init();


               // sbus::init();
               // sbus_uart::init();
               // sbus_uart::baud(100'000);
               // sbus_uart::parity(true);
               // sbus_uart::invert(true);
               // // sbus_uart::template halfDuplex<true>();
               // // sbus_uart::rxtxswap(true);
               // sbustx::afunction(7);
               // sbustx::template pulldown<true>();

        led::template dir<Mcu::Output>();

        // tp::template dir<Mcu::Output>();
        dac::init();

        isense::analog();
        vsense::analog();
        pga::init();
        pga::input(1);
        pga::gain(1); // 4
        adcDmaChannel1::init();
        adc::init();

        pwm::init();
        fault::pullup();
        fault::template dir<Mcu::Input>();

        disable::template dir<Mcu::Output>();
        disable::reset();
        in1::template dir<Mcu::Output>();
        in2::template dir<Mcu::Output>();
        // in1::afunction(2);
        // in2::afunction(2);
        nsleep::template dir<Mcu::Output>();
        nsleep::set();
        nsleepPulseWaiter::init();

        tp::set();
        tp::reset();
    }
};

template<typename Config>
struct Devices<ESC02, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 1'000_Hz, Mcu::Stm::HSI>, MCU>;
    // using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<60_MHz, 30'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 2_MHz, 128>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    // PA2 Telem : USART2-TX, AF7

    // PA9 Bus : USART1-TX, AF7

    struct SbusCallback;
    using sbus_pa = RC::Protokoll::SBus2::Adapter<0, SbusCallback>;
    using sbus_uart = Mcu::Stm::Uart<2, sbus_pa, 32, std::byte, clock, MCU>;
    using sbus = RC::Protokoll::SBus2::Fsm<sbus_uart, systemTimer>;
    using sbustx = Mcu::Stm::Pin<gpiob, 4, MCU>; // rx, AF 7, need swap rxtx-pins


    // PB7 Led
    using led = Mcu::Stm::Pin<gpiob, 7, MCU>;

    // PB4 Fault
    // PA15 nSleep : TIM2-CH1
    // PA6 Pwm1 : TIM3-CH1, AF2
    // PA4 Pwm2 : TIM3-CH2, AF2
    //            TIM3-CH3 as trgo
    // PB0 Disable

    using pwm = Mcu::Stm::Motor::Bdc<3, clock, MCU>;
    using in1 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using in2 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using disable = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using nsleep = Mcu::Stm::Pin<gpioa, 15, MCU>;
    using fault = Mcu::Stm::Pin<gpiob, 4, MCU>;
    using nsleepPulseWaiter = Mcu::Stm::Waiter<6,
                                               std::integral_constant<uint16_t, 35 * 170>,
                                               std::integral_constant<uint16_t, 0>, MCU>;

    // besser auch die PGA genutzt
    // PA0 ISense : ADC12-IN1
    // PA3 VInSense : ADC1-IN4
    // using adc = Mcu::Stm::Adc<1, 4, Mcu::Stm::NoTriggerSource, Mcu::Stm::UseDma<false>, MCU>;
    using adc = Mcu::Stm::V2::Adc<1, 4, pwm, Mcu::Stm::UseDma<false>, MCU>;
    using isense = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using vsense = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pf0 = Mcu::Stm::Pin<gpiof, 0, MCU>;

    // PA1 Comparator Lmt01

    // PA5 Testpunkt, DAC1-Out2

    using tp = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using dac = Mcu::Stm::Dac<1, MCU>;

    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        gpioa::init();
        gpiob::init();
        gpioc::init();
        gpiof::init();


        // sbus::init();
        // sbus_uart::init();
        // sbus_uart::baud(100'000);
        // sbus_uart::parity(true);
        // sbus_uart::invert(true);
        // // sbus_uart::template halfDuplex<true>();
        // // sbus_uart::rxtxswap(true);
        // sbustx::afunction(7);
        // sbustx::template pulldown<true>();

        led::template dir<Mcu::Output>();

        tp::template dir<Mcu::Output>();
        // dac::init();

        isense::analog();
        vsense::analog();
        pf0::analog();
        adc::init();

        pwm::init();
        fault::pullup();
        fault::template dir<Mcu::Input>();

        disable::template dir<Mcu::Output>();
        disable::reset();
        in1::template dir<Mcu::Output>();
        in2::template dir<Mcu::Output>();
        // in1::afunction(2);
        // in2::afunction(2);
        nsleep::template dir<Mcu::Output>();
        nsleep::set();
        nsleepPulseWaiter::init();

        tp::set();
        tp::reset();
    }
};

template<typename Config>
struct Devices<ESC01, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    //    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 1'000_Hz, Mcu::Stm::HSI>, MCU>;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<60_MHz, 30'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 2_MHz, 128>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    // PA2 Telem : USART2-TX, AF7

    // PA9 Bus : USART1-TX, AF7

    struct SbusCallback;
    using sbus_pa = RC::Protokoll::SBus2::Adapter<0, SbusCallback>;
    using sbus_uart = Mcu::Stm::Uart<2, sbus_pa, 32, std::byte, clock, MCU>;
    using sbus = RC::Protokoll::SBus2::Fsm<sbus_uart, systemTimer>;
    using sbustx = Mcu::Stm::Pin<gpiob, 4, MCU>; // rx, AF 7, need swap rxtx-pins


    // PB7 Led
    using led = Mcu::Stm::Pin<gpiob, 7, MCU>;

    // PB4 Fault
    // PA15 nSleep : TIM2-CH1
    // PA6 Pwm1 : TIM3-CH1, AF2
    // PA4 Pwm2 : TIM3-CH2, AF2
    // PB0 Disable

    using pwm = Mcu::Stm::Motor::Bdc<3, clock, MCU>;
    using in1 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using in2 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using disable = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using nsleep = Mcu::Stm::Pin<gpioa, 15, MCU>;
    using fault = Mcu::Stm::Pin<gpiob, 4, MCU>;

    // besser auch die PGA genutzt
    // PA0 ISense : ADC12-IN1
    // PA3 VInSense : ADC1-IN4
    using adc = Mcu::Stm::V2::Adc<1, 1, pwm, Mcu::Stm::UseDma<false>, MCU>; // opamp1 out


    // PA1 Comparator Lmt01

    // PA5 Testpunkt, DAC1-Out2

    using tp = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using dac = Mcu::Stm::Dac<1, MCU>;

    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();

        gpioa::init();
        gpiob::init();
        gpioc::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        sbus::init();
        sbus_uart::init();
        sbus_uart::baud(100'000);
        sbus_uart::parity(true);
        sbus_uart::invert(true);
        // sbus_uart::template halfDuplex<true>();
        // sbus_uart::rxtxswap(true);
        sbustx::afunction(7);
        sbustx::template pulldown<true>();

        led::template dir<Mcu::Output>();

        tp::template dir<Mcu::Output>();
        dac::init();

        pwm::init();
        fault::pullup();
        fault::template dir<Mcu::Input>();

        disable::template dir<Mcu::Output>();
        disable::reset();
        in1::template dir<Mcu::Output>();
        in2::template dir<Mcu::Output>();
        // in1::afunction(2);
        // in2::afunction(2);
        nsleep::template dir<Mcu::Output>();
        nsleep::set();
    }
};

#endif
