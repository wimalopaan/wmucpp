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
struct ESC02; // mit adc trigger über timer3 (geht nicht direkt)
struct ESC03; // mit adc trigger über timer3 isr
struct ESC04; // mit adc trigger über timer3 isr und dma

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
struct Devices<ESC04, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>;
    // using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<60_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>;
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
    using ibus_sensor = RC::Protokoll::IBus::Sensor<ibus_uart, systemTimer, Meta::List<RpmProvider, CurrentProvider, VoltageProvider>>;
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
    using nsleepPulseWaiter = Mcu::Stm::Waiter<6,
                                               std::integral_constant<uint16_t, 35 * 170>,
                                               std::integral_constant<uint16_t, 0>, MCU>;

           // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

           // besser auch die PGA genutzt
           // PA0 ISense : ADC12-IN1 (auf Platine geändert -> VSense)
           // PA3 VInSense : ADC1-IN4 (auf Platine geändert -> ISense)

    using adcDmaChannel = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using adcDmaStorage = std::array<volatile uint16_t, 2>;
    using adc = Mcu::Stm::V3::Adc<1, Meta::NList<13, 1>, pwm, adcDmaChannel, adcDmaStorage, MCU>; // opamp1
    // using adc = Mcu::Stm::V3::Adc<1, Meta::NList<13, 1>, Mcu::Stm::NoTriggerSource, adcDmaChannel, adcDmaStorage, MCU>; // opamp1
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
