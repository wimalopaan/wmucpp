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

struct ESC01; // nur pwm
// struct ESC02; // mit adc trigger über timer3 (geht nicht direkt)
// struct ESC03; // mit adc trigger über timer3 isr
// struct ESC04; // mit adc trigger über timer3 isr und dma, ibus
struct ESC05; // mit adc trigger über timer3, dma, ibus, variable pwm Frequenz

struct ESC10; // adc-isr -> sub-sampling

using namespace Mcu::Stm;
using namespace std::literals::chrono_literals;

template<typename HW, typename Config, typename MCU = void>
struct Devices;

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
