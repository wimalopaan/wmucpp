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

#include "../include/comparator.h"
#include "../include/crsf_cb.h"
#include "../include/telemetry.h"

struct ESC01; // nur pwm
// struct ESC02; // mit adc trigger über timer3 (geht nicht direkt)
// struct ESC03; // mit adc trigger über timer3 isr
// struct ESC04; // mit adc trigger über timer3 isr und dma, ibus
struct ESC05; // mit adc trigger über timer3, dma, ibus, variable pwm Frequenz

struct ESC10; // adc-isr -> sub-sampling

struct ESC20;

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
    using trace = Arm::Trace<clock, 10_MHz, 2048>;
#endif

    using store = Config::storage;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    static inline constexpr float adc2Voltage(const auto a) {
        // constexpr float r1 = 91'000;
        constexpr float r1 = 100'000; // watch out for assembly!
        constexpr float r2 = 10'000;
        constexpr float vref = 3.3f;
        constexpr float max = 4095.0f;
        // constexpr float cal = 0.94;
        const float cal = (store::eeprom.calib_ubatt - 100) * 0.001f + 1.0f;

        return cal * (a * vref) * (r1 + r2) / (max * r2);
    }
    static inline constexpr float adc2Current(const auto i) {
        constexpr float vref = 3.3f;
        constexpr float max = 4095.0f;
        constexpr float rs = 0.001f; // 1mOhm Shunt
        constexpr float gain = 50.0f; // INA240A2 G=50
        return (i * vref) / (max * rs * gain);
    }

    // PA9 Bus : USART1-TX, AF7, rxtx swap
    struct CrsfAdapterConfig;
    using servo_pa = RC::Protokoll::Crsf::V2::Adapter<0, CrsfAdapterConfig, MCU>;
    using servo = Mcu::Stm::Uart<1, servo_pa, 0, char, clock, MCU>;
    using servotx = Mcu::Stm::Pin<gpioa, 9, MCU>;

    // PA15 Telem: USART2-RX, AF7, rxtx swap
    using sensor_uart = Mcu::Stm::Uart<2, void, 128, std::byte, clock, MCU>;
    using sensorrxtx = Mcu::Stm::Pin<gpioa, 15, MCU>;
    using crsf_out = RC::Protokoll::Crsf::V2::Generator<sensor_uart, systemTimer, MCU>;

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

           // PA10 Led
    using led = Mcu::Stm::Pin<gpioa, 10, MCU>;
    using ledBlinker = External::Blinker<led, systemTimer>;

    // PA6 Pwm1 : TIM3-CH1, AF2 half-bridge1 LS
    // PA7 Pwm2 : TIM3-CH2, AF2 half-bridge1 HS
    // PB0 Pwm3 : TIM3-CH3, AF2 half-bridge2 LS
    // PB7 Pwm4 : TIM3-CH4, AF10 half-bridge2 HS

    using pwm1 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using pwm2 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using pwm3 = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using pwm4 = Mcu::Stm::Pin<gpiob, 7, MCU>;
    using pwm = Mcu::Stm::Motor::Identification::BdcWithPins4<3, 4, pwm1, pwm2, pwm3, pwm4, clock, MCU>;

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

    // PA0 VInSense : ADC2-IN1
    // PA1 ISense : OpAmp3-VinP ADC2-IN18
    // Temp: ADC1-IN16

    using adcDmaChannel = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using adcDmaStorage = std::array<volatile uint16_t, 3>;
    // using adc = Mcu::Stm::V3::Adc<2, Meta::NList<18, 1>, pwm, adcDmaChannel, adcDmaStorage, Meta::List<EndOfSequence>, MCU>; // opamp3
    using adc = Mcu::Stm::V3::Adc<1, Meta::NList<13, 1, 16>, pwm, adcDmaChannel, adcDmaStorage, Meta::List<EndOfSequence>, MCU>; // opamp1
    using isense = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using vsense = Mcu::Stm::Pin<gpioa, 0, MCU>;
    // using pga = Mcu::Stm::PGA<3, MCU>;
    using pga = Mcu::Stm::PGA<1, MCU>;

    // PA3 Comparator Lmt01
    // using compinp = Mcu::Stm::Pin<gpioa, 3, MCU>;

    // PA2 OpAmp1-Out: Offset
    // DAC3-Out1 -> OpAmp1-Vinp
    // using offset = Mcu::Stm::Dac<3, MCU>;
    // using follow1 = Mcu::Stm::Follower<1, MCU>;
    using offset = Mcu::Stm::Pin<gpioa, 2, MCU>;

    // PA4 Testpunkt-0, DAC1-Out1
    // PA11 Testpunkt-1
    // PA12 Testpunkt-2

    using tp0 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using tp1 = Mcu::Stm::Pin<gpioa, 11, MCU>;
    using tp2 = Mcu::Stm::Pin<gpioa, 12, MCU>;

    // using dac = Mcu::Stm::Dac<1, MCU>;
    // using comp1 = Mcu::Stm::Comparator<1, compinp, Mcu::Stm::VRefDiv<3, 4>, tp2>;

    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        gpioa::init();
        gpiob::init();
        gpioc::init();
        gpiof::init();

        led::template dir<Mcu::Output>();

        // tp0::template dir<Mcu::Output>();
        tp1::template dir<Mcu::Output>();
        tp2::template dir<Mcu::Output>();

        // dac::init();

        // offset::init();
        // offset::set(0);
        // follow1::init();
        offset::template dir<Mcu::Output>();

        isense::analog();
        vsense::analog();
        pga::init();
        // pga::input(2);
        pga::input(0);
        pga::gain(0);
        adc::init();

        pwm::init();
        pwm::duty(0);

        servotx::afunction(7);
        servotx::pullup();
        servo::init();
        servo::baud(420'000);
        servo::rxtxswap(true);

        sensorrxtx::afunction(7);
        // sensorrxtx::pullup();
        // sensorrxtx::openDrain();
        sensor_uart::init();
        sensor_uart::baud(420'000);
        sensor_uart::rxtxswap(true);

        // comp1::init();
        // comp1::enableInt();

        tp2::set();
        tp2::reset();
    }
};
#endif

#ifndef USE_DEVICES2

struct RpmProvider {
    inline static constexpr auto ibus_type = RC::Protokoll::IBus::Type::type::RPM;
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
        return mValue;
    }
    inline static uint16_t mValue{};
};
struct CurrentProvider {
    inline static constexpr auto ibus_type = RC::Protokoll::IBus::Type::type::BAT_CURR;
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
        return mValue;
    }
    inline static uint16_t mValue{};
};
struct VoltageProvider {
    inline static constexpr auto ibus_type = RC::Protokoll::IBus::Type::type::EXTERNAL_VOLTAGE;
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
        return mValue;
    }
    inline static uint16_t mValue{};
};

template<typename Config>
struct Devices<ESC10, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 2_MHz, 128>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    // PA15 Telem : USART2-RX, AF7, swap rxtx, half-duplex
    // PA9 Bus : USART1-TX, AF7

    // struct SbusCallback;
    // using sbus_pa = RC::Protokoll::SBus2::Adapter<0, SbusCallback>;
    // using sbus_uart = Mcu::Stm::Uart<2, sbus_pa, 32, std::byte, clock, MCU>;
    // using sbus = RC::Protokoll::SBus2::Fsm<sbus_uart, systemTimer>;
    // using sbustx = Mcu::Stm::Pin<gpioa, 9, MCU>; // rx, AF 7, need swap rxtx-pins

    using servo_pa = RC::Protokoll::IBus::Adapter<0, void>;
    using servo = Mcu::Stm::Uart<1, servo_pa, 0, char, clock, MCU>;
    using servotx = Mcu::Stm::Pin<gpioa, 9, MCU>;

    template<typename PA>
    using ibus_uart = Mcu::Stm::Uart<2, PA, 16, std::byte, clock, MCU>;
    using ibus_sensor = RC::Protokoll::IBus::Sensor<ibus_uart, systemTimer, Meta::List<RpmProvider, CurrentProvider, VoltageProvider>, void, true, true>;
    using ibusrxtx = Mcu::Stm::Pin<gpioa, 15, MCU>;

           // PA10 Led
    using led = Mcu::Stm::Pin<gpioa, 10, MCU>;

           // PA6 Pwm1 : TIM3-CH1, AF2
           // PA7 Pwm2 : TIM3-CH2, AF2
           // PB0 Pwm3 : TIM3-CH3, AF2
           // PB7 Pwm4 : TIM3-CH4, AF2

    using pwm = Mcu::Stm::Motor::Bdc2<3, clock, MCU>;
    using pwm1 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using pwm2 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using pwm3 = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using pwm4 = Mcu::Stm::Pin<gpiob, 7, MCU>;

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

    // PA0 VInSense : ADC2-IN1
    // PA1 ISense : OpAmp3-VinP

    using adcDmaChannel = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using adcDmaStorage = std::array<volatile uint16_t, 2>;
    using adc = Mcu::Stm::V3::Adc<2, Meta::NList<18, 1>, pwm, adcDmaChannel, adcDmaStorage, Meta::List<EndOfSequence>, MCU>; // opamp1
    using isense = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using vsense = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using pga = Mcu::Stm::PGA<3, MCU>;

    // PA1 Comparator Lmt01

    // PA2 OpAmp1-Out: Offset
    // DAC3-Out1 -> OpAmp1-Vinp
    using offset = Mcu::Stm::Dac<3, MCU>;
    using follow1 = Mcu::Stm::Follower<1, MCU>;

    // PA4 Testpunkt-0, DAC1-Out1
    // PA11 Testpunkt-1
    // PA12 Testpunkt-2

    // using tp0 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using tp1 = Mcu::Stm::Pin<gpioa, 11, MCU>;
    using tp2 = Mcu::Stm::Pin<gpioa, 12, MCU>;

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
        // tp0::template dir<Mcu::Output>();
        tp1::template dir<Mcu::Output>();
        tp2::template dir<Mcu::Output>();

        dac::init();

        offset::init();
        offset::set(0);
        follow1::init();

        isense::analog();
        vsense::analog();
        pga::init();
        pga::input(2);
        pga::gain(1); // 4
        adc::init();

        pwm1::template dir<Mcu::Output>();
        pwm2::template dir<Mcu::Output>();
        pwm3::template dir<Mcu::Output>();
        pwm4::template dir<Mcu::Output>();

        pwm::init();
        pwm1::afunction(2);
        pwm2::afunction(2);
        // pwm3::afunction(2);
        // pwm4::afunction(2);


        servotx::afunction(7);
        servotx::pullup();
        servo::init();
        servo::baud(115'200);
        servo::rxtxswap(true);

        ibusrxtx::afunction(7);
        ibusrxtx::pullup();
        ibusrxtx::openDrain();
        ibus_sensor::init();

        tp1::set();
        tp1::reset();
    }
};


template<typename Config>
struct Devices<ESC05, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 2_MHz, 128>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    // PA15 Telem : USART2-RX, AF7, swap rxtx, half-duplex
    // PA9 Bus : USART1-TX, AF7

    // struct SbusCallback;
    // using sbus_pa = RC::Protokoll::SBus2::Adapter<0, SbusCallback>;
    // using sbus_uart = Mcu::Stm::Uart<2, sbus_pa, 32, std::byte, clock, MCU>;
    // using sbus = RC::Protokoll::SBus2::Fsm<sbus_uart, systemTimer>;
    // using sbustx = Mcu::Stm::Pin<gpioa, 9, MCU>; // rx, AF 7, need swap rxtx-pins

    using servo_pa = RC::Protokoll::IBus::Adapter<0, void>;
    using servo = Mcu::Stm::Uart<1, servo_pa, 0, char, clock, MCU>;
    using servotx = Mcu::Stm::Pin<gpioa, 9, MCU>;

    template<typename PA>
    using ibus_uart = Mcu::Stm::Uart<2, PA, 16, std::byte, clock, MCU>;
    using ibus_sensor = RC::Protokoll::IBus::Sensor<ibus_uart, systemTimer, Meta::List<RpmProvider, CurrentProvider, VoltageProvider>, void, true, true>;
    using ibusrxtx = Mcu::Stm::Pin<gpioa, 15, MCU>;

           // PA10 Led
    using led = Mcu::Stm::Pin<gpioa, 10, MCU>;

           // PA6 Pwm1 : TIM3-CH1, AF2
           // PA7 Pwm2 : TIM3-CH2, AF2
           // PB0 Pwm3 : TIM3-CH3, AF2
           // PB7 Pwm4 : TIM3-CH4, AF2

    using pwm = Mcu::Stm::Motor::Bdc2<3, clock, MCU>;
    using pwm1 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using pwm2 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using pwm3 = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using pwm4 = Mcu::Stm::Pin<gpiob, 7, MCU>;

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

    // PA0 VInSense : ADC2-IN1
    // PA1 ISense : OpAmp3-VinP

    using adcDmaChannel = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using adcDmaStorage = std::array<volatile uint16_t, 2>;
    using adc = Mcu::Stm::V3::Adc<2, Meta::NList<18, 1>, pwm, adcDmaChannel, adcDmaStorage, void, MCU>; // opamp1
    using isense = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using vsense = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using pga = Mcu::Stm::PGA<3, MCU>;

    // PA1 Comparator Lmt01

    // PA2 OpAmp1-Out: Offset
    // DAC3-Out1 -> OpAmp1-Vinp
    using offset = Mcu::Stm::Dac<3, MCU>;
    using follow1 = Mcu::Stm::Follower<1, MCU>;

    // PA4 Testpunkt-0, DAC1-Out1
    // PA11 Testpunkt-1
    // PA12 Testpunkt-2

    // using tp0 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using tp1 = Mcu::Stm::Pin<gpioa, 11, MCU>;
    using tp2 = Mcu::Stm::Pin<gpioa, 12, MCU>;

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
        // tp0::template dir<Mcu::Output>();
        tp1::template dir<Mcu::Output>();
        tp2::template dir<Mcu::Output>();

        dac::init();

        offset::init();
        offset::set(0);
        follow1::init();

        isense::analog();
        vsense::analog();
        pga::init();
        pga::input(2);
        pga::gain(1); // 4
        adc::init();

        pwm1::template dir<Mcu::Output>();
        pwm2::template dir<Mcu::Output>();
        pwm3::template dir<Mcu::Output>();
        pwm4::template dir<Mcu::Output>();

        pwm::init();
        pwm1::afunction(2);
        pwm2::afunction(2);
        // pwm3::afunction(2);
        // pwm4::afunction(2);


        servotx::afunction(7);
        servotx::pullup();
        servo::init();
        servo::baud(115'200);
        servo::rxtxswap(true);

        ibusrxtx::afunction(7);
        ibusrxtx::pullup();
        ibusrxtx::openDrain();
        ibus_sensor::init();

        tp1::set();
        tp1::reset();
    }
};

template<typename Config>
struct Devices<ESC01, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 1'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 2_MHz, 128>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    // PA15 Telem : USART2-RX, AF7, swap rxtx, half-duplex
    // PA9 Bus : USART1-TX, AF7

    struct SbusCallback;
    using sbus_pa = RC::Protokoll::SBus2::Adapter<0, SbusCallback>;
    using sbus_uart = Mcu::Stm::Uart<2, sbus_pa, 32, std::byte, clock, MCU>;
    using sbus = RC::Protokoll::SBus2::Fsm<sbus_uart, systemTimer>;
    using sbustx = Mcu::Stm::Pin<gpioa, 9, MCU>; // tx, AF 7, need swap rxtx-pins

    // PA10 Led
    using led = Mcu::Stm::Pin<gpioa, 10, MCU>;

    // PA6 Pwm1 : TIM3-CH1, AF2
    // PA7 Pwm2 : TIM3-CH2, AF2
    // PB0 Pwm3 : TIM3-CH3, AF2
    // PB7 Pwm4 : TIM3-CH4, AF2

    using pwm = Mcu::Stm::Motor::Bdc2<3, clock, MCU>;
    using pwm1 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using pwm2 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using pwm3 = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using pwm4 = Mcu::Stm::Pin<gpiob, 7, MCU>;

    // besser auch die PGA genutzt
    // PA0 VInSense : ADC2-IN1
    // PA1 ISense : OpAmp3-VinP
    using adc = Mcu::Stm::V2::Adc<1, 1, pwm, Mcu::Stm::UseDma<false>, MCU>; // opamp1 out

    // PA2 Offset : OpAmp1Vout

    // PA3 Comp2Inp2 Lmt01

    // PA4 Testpunkt-0, DAC1-Out1
    // PA11 Testpunkt-1
    // PA12 Testpunkt-2

    using tp0 = Mcu::Stm::Pin<gpioa, 4, MCU>;
    using tp1 = Mcu::Stm::Pin<gpioa, 11, MCU>;
    using tp2 = Mcu::Stm::Pin<gpioa, 12, MCU>;

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

        tp0::template dir<Mcu::Output>();
        dac::init();

        pwm1::template dir<Mcu::Output>();
                         pwm2::template dir<Mcu::Output>();
                         pwm3::template dir<Mcu::Output>();
                         pwm4::template dir<Mcu::Output>();

        pwm::init();
        // pwm::duty(1638);
        pwm1::afunction(2);
        pwm2::afunction(2);
        // pwm3::afunction(2);
        // pwm4::afunction(2);

        // pwm3::template dir<Mcu::Output>();
        // pwm3::set();
    }
};

#endif
