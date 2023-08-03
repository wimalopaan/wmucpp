#pragma once

#include "mcu.h"
#include "rc.h"
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
#include "mcu_traits.h"
#include "arm.h"
#include "gpio.h"
#include "tick.h"

struct FSK;
struct FSK01;
struct Test01;
struct SDR01;
struct SDR02;
struct SDR03;
struct SDR04;

template<typename HW, typename MCU = void>
struct Devices;

template<>
struct Devices<SDR04, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 48'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 
//    using systemTimer = STM32::SystemTimer<clock, UseInterrupts<true>, MCU>; 
//    using trace = Arm::Trace<clock>; // 2MHz ITM Takt
    using trace = Arm::Trace<clock, 24_MHz, 128>; 

    using adc_period = std::integral_constant<uint16_t, 170'000'000 / 48'000>;
    using adc_prescaler = std::integral_constant<uint16_t, 0>; // pre=1
    using adc_timer = Mcu::Stm::Timer<6, adc_period, adc_prescaler, Mcu::Stm::Trigger<true>, MCU>;
    
    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    using pina0 = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using pina1 = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using pina2 = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using pina3 = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pina8 = Mcu::Stm::Pin<gpioa, 8, MCU>;
    using sda = pina8;
    using pina9 = Mcu::Stm::Pin<gpioa, 9, MCU>;
    using scl = pina9;
    using pina11 = Mcu::Stm::Pin<gpioa, 11, MCU>;
    
    using pinb0 = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using pinb4 = Mcu::Stm::Pin<gpiob, 4, MCU>; // changed to pa11, now af2
    using pinb5 = Mcu::Stm::Pin<gpiob, 5, MCU>;

    using pintx = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using pinrx = Mcu::Stm::Pin<gpiob, 7, MCU>;

    using serial1 = Mcu::Stm::Uart<1, RC::Protokoll::Null::Adapter<>, 64, char, clock, MCU>;
    
    using led = Mcu::Stm::Pin<gpiob, 8, MCU>;

    using pinf0 = Mcu::Stm::Pin<gpiof, 0, MCU>;
    using pinf1 = Mcu::Stm::Pin<gpiof, 1, MCU>;
    
    using dac1 = Mcu::Stm::Dac<1, MCU>;
    using dac3 = Mcu::Stm::Dac<3, MCU>;
    
    using adc1 = Mcu::Stm::Adc<1, 13, adc_timer, MCU>; // opamp1 out
    using adc2 = Mcu::Stm::Adc<2, 16, adc_timer, MCU>; // opamp2 out

//    using pga1 = Mcu::Stm::PGA<1, MCU>;
    using pga2 = Mcu::Stm::PGA<2, MCU>;
    
    using follow1 = Mcu::Stm::Follower<1, MCU>;
    
    using cppm_period = std::integral_constant<uint16_t, 5120>;
    using cppm_prescaler = std::integral_constant<uint16_t, 664>;
    
    using pwm1 = Mcu::Stm::Timer<3, cppm_period, cppm_prescaler, Mcu::Stm::Trigger<false>, MCU>;
//    using pina6 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using pwm2 = Mcu::Stm::Timer<4, cppm_period, cppm_prescaler, Mcu::Stm::Trigger<false>, MCU>;
    
    using i2c2 = Mcu::Stm::I2C::Master<2, 16, MCU>;
    static inline constexpr Mcu::Stm::I2C::Address si5351{0x60};
    using si = External::SI5351::IQClock<i2c2, si5351, External::SI5351::IOutput<0>, External::SI5351::IOutput<1>>;
    
    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();
        adc_timer::init();
        
        gpioa::init();
        gpiob::init();
        gpiof::init();

        pinb0::template dir<Mcu::Output>();
        pinb0::template speed<Mcu::High>();
        pinb5::template dir<Mcu::Output>();
        pinb5::template speed<Mcu::High>();

//        pinb6::template dir<Mcu::Output>();
//        pinb6::template speed<Mcu::High>();

        pintx::afunction(7);
        pinrx::afunction(7);
        pinrx::pullup<true>();
        
        led::template dir<Mcu::Output>();

        pina11::template dir<Mcu::Output>();
        pina11::template speed<Mcu::High>();
        
//        pina0::analog();
//        pina1::analog();
//        pina2::analog();
//        pina3::analog();

//        pga1::init();
//        pga1::gain(0x0a);
        
        pga2::init();
        pga2::gain(0x08); // x2
        pga2::gain(0x09); // x4
//        pga2::gain(0x0a); // x8
//        pga2::gain(0x0b); // x16
//        pga2::gain(0x0c); // x32
        
        follow1::init();
        
        dac1::init();
        dac3::init();
        
//        adc1::init();
        
        adc2::init();
//        adc2::trigger(13); // Timer 6
        
//        pinb4::template dir<Mcu::Output>();
//        pinb4::template speed<Mcu::High>();
        pinb4::afunction(2);
        pwm1::init();
        
        sda::openDrain();
        scl::openDrain();
        sda::afunction(4);
        scl::afunction(4);
        
        i2c2::init();
        
        serial1::init();
        serial1::baud(9600);
    }
};

template<>
struct Devices<SDR03, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 48'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 
//    using systemTimer = STM32::SystemTimer<clock, UseInterrupts<true>, MCU>; 
//    using trace = Arm::Trace<clock>; // 2MHz ITM Takt
    using trace = Arm::Trace<clock, 24_MHz, 128>; 

    using adc_period = std::integral_constant<uint16_t, 170'000'000 / 48'000>;
//    using adc_period = std::integral_constant<uint16_t, 2000>;
    using adc_prescaler = std::integral_constant<uint16_t, 0>; // pre=1
    
    using adc_timer = Mcu::Stm::Timer<6, adc_period, adc_prescaler, Mcu::Stm::Trigger<true>, MCU>;
    
    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    using pina0 = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using pina1 = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using pina2 = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using pina3 = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pina8 = Mcu::Stm::Pin<gpioa, 8, MCU>;
    using sda = pina8;
    using pina9 = Mcu::Stm::Pin<gpioa, 9, MCU>;
    using scl = pina9;
    using pina11 = Mcu::Stm::Pin<gpioa, 11, MCU>;
    
    using pinb0 = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using pinb4 = Mcu::Stm::Pin<gpiob, 4, MCU>; // changed to pa11, now af2
    using pinb5 = Mcu::Stm::Pin<gpiob, 5, MCU>;

    using pintx = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using pinrx = Mcu::Stm::Pin<gpiob, 7, MCU>;

    using serial1 = Mcu::Stm::Uart<1, RC::Protokoll::Null::Adapter<>, 64, char, clock, MCU>;
    
    using led = Mcu::Stm::Pin<gpiob, 8, MCU>;

    using pinf0 = Mcu::Stm::Pin<gpiof, 0, MCU>;
    using pinf1 = Mcu::Stm::Pin<gpiof, 1, MCU>;
    
    using dac1 = Mcu::Stm::Dac<1, MCU>;
    using dac3 = Mcu::Stm::Dac<3, MCU>;
    
//    using adc1 = Mcu::Stm::Adc<1, 13, MCU>; // opamp1 out
    using adc2 = Mcu::Stm::Adc<2, 16, adc_timer, MCU>; // opamp2 out

//    using pga1 = Mcu::Stm::PGA<1, MCU>;
    using pga2 = Mcu::Stm::PGA<2, MCU>;
    
    using follow1 = Mcu::Stm::Follower<1, MCU>;
    
    using cppm_period = std::integral_constant<uint16_t, 5120>;
    using cppm_prescaler = std::integral_constant<uint16_t, 664>;
    
    using pwm1 = Mcu::Stm::Timer<3, cppm_period, cppm_prescaler, Mcu::Stm::Trigger<false>, MCU>;
//    using pina6 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using pwm2 = Mcu::Stm::Timer<4, cppm_period, cppm_prescaler, Mcu::Stm::Trigger<false>, MCU>;
    
    using i2c2 = Mcu::Stm::I2C::Master<2, 16, MCU>;
    static inline constexpr Mcu::Stm::I2C::Address si5351{0x60};
    using si = External::SI5351::Clock<i2c2, si5351>;
    
    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();
        adc_timer::init();
        
        gpioa::init();
        gpiob::init();
        gpiof::init();

        pinb0::template dir<Mcu::Output>();
        pinb0::template speed<Mcu::High>();
        pinb5::template dir<Mcu::Output>();
        pinb5::template speed<Mcu::High>();

//        pinb6::template dir<Mcu::Output>();
//        pinb6::template speed<Mcu::High>();

        pintx::afunction(7);
        pinrx::afunction(7);
        pinrx::pullup<true>();
        
        led::template dir<Mcu::Output>();

        pina11::template dir<Mcu::Output>();
        pina11::template speed<Mcu::High>();
        
//        pina0::analog();
//        pina1::analog();
//        pina2::analog();
//        pina3::analog();

//        pga1::init();
//        pga1::gain(0x0a);
        
        pga2::init();
        pga2::gain(0x08); // x2
        pga2::gain(0x09); // x4
//        pga2::gain(0x0a); // x8
//        pga2::gain(0x0b); // x16
//        pga2::gain(0x0c); // x32
        
        follow1::init();
        
        dac1::init();
        dac3::init();
        
//        adc1::init();
        
        adc2::init();
//        adc2::trigger(13); // Timer 6
        
//        pinb4::template dir<Mcu::Output>();
//        pinb4::template speed<Mcu::High>();
        pinb4::afunction(2);
        pwm1::init();
        
        sda::openDrain();
        scl::openDrain();
        sda::afunction(4);
        scl::afunction(4);
        
        i2c2::init();
        
        serial1::init();
        serial1::baud(9600);
    }
};

template<>
struct Devices<SDR02, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 48'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 
//    using systemTimer = STM32::SystemTimer<clock, UseInterrupts<true>, MCU>; 
//    using trace = Arm::Trace<clock>; // 2MHz ITM Takt
    using trace = Arm::Trace<clock, 24_MHz>; 

    using adc_period = std::integral_constant<uint16_t, 170'000'000 / 48'000>;
//    using adc_period = std::integral_constant<uint16_t, 2000>;
    using adc_prescaler = std::integral_constant<uint16_t, 0>; // pre=1
    
    using adc_timer = Mcu::Stm::Timer<6, adc_period, adc_prescaler, Mcu::Stm::Trigger<true>, MCU>;
    
    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    using pina0 = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using pina1 = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using pina2 = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using pina3 = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pina8 = Mcu::Stm::Pin<gpioa, 8, MCU>;
    using sda = pina8;
    using pina9 = Mcu::Stm::Pin<gpioa, 9, MCU>;
    using scl = pina9;
    using pina11 = Mcu::Stm::Pin<gpioa, 11, MCU>;
    
    using pinb0 = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using pinb4 = Mcu::Stm::Pin<gpiob, 4, MCU>; // changed to pa11, now af2
    using pinb5 = Mcu::Stm::Pin<gpiob, 5, MCU>;
    using pinb6 = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using led = Mcu::Stm::Pin<gpiob, 8, MCU>;

    using pinf0 = Mcu::Stm::Pin<gpiof, 0, MCU>;
    using pinf1 = Mcu::Stm::Pin<gpiof, 1, MCU>;
    
    using dac1 = Mcu::Stm::Dac<1, MCU>;
    using dac3 = Mcu::Stm::Dac<3, MCU>;
    
//    using adc1 = Mcu::Stm::Adc<1, 13, MCU>; // opamp1 out
    using adc2 = Mcu::Stm::Adc<2, 16, adc_timer, MCU>; // opamp2 out

//    using pga1 = Mcu::Stm::PGA<1, MCU>;
    using pga2 = Mcu::Stm::PGA<2, MCU>;
    
    using follow1 = Mcu::Stm::Follower<1, MCU>;
    
    using cppm_period = std::integral_constant<uint16_t, 5120>;
    using cppm_prescaler = std::integral_constant<uint16_t, 664>;
    
    using pwm1 = Mcu::Stm::Timer<3, cppm_period, cppm_prescaler, Mcu::Stm::Trigger<false>, MCU>;
//    using pina6 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using pwm2 = Mcu::Stm::Timer<4, cppm_period, cppm_prescaler, Mcu::Stm::Trigger<false>, MCU>;
    
    using i2c2 = Mcu::Stm::I2C::Master<2, 16, MCU>;
    static inline constexpr Mcu::Stm::I2C::Address si5351{0x60};
    using si = External::SI5351::Clock<i2c2, si5351>;
    
    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();
        adc_timer::init();
        
        gpioa::init();
        gpiob::init();
        gpiof::init();

        pinb0::template dir<Mcu::Output>();
        pinb0::template speed<Mcu::High>();
        pinb5::template dir<Mcu::Output>();
        pinb5::template speed<Mcu::High>();
        pinb6::template dir<Mcu::Output>();
        pinb6::template speed<Mcu::High>();
        led::template dir<Mcu::Output>();

        pina11::template dir<Mcu::Output>();
        pina11::template speed<Mcu::High>();
        
//        pina0::analog();
//        pina1::analog();
//        pina2::analog();
//        pina3::analog();

//        pga1::init();
//        pga1::gain(0x0a);
        
        pga2::init();
        pga2::gain(0x08); // x2
        pga2::gain(0x09); // x4
//        pga2::gain(0x0a); // x8
//        pga2::gain(0x0b); // x16
//        pga2::gain(0x0c); // x32
        
        follow1::init();
        
        dac1::init();
        dac3::init();
        
//        adc1::init();
        
        adc2::init();
//        adc2::trigger(13); // Timer 6
        
//        pinb4::template dir<Mcu::Output>();
//        pinb4::template speed<Mcu::High>();
        pinb4::afunction(2);
        pwm1::init();
        
        sda::openDrain();
        scl::openDrain();
        sda::afunction(4);
        scl::afunction(4);
        
        i2c2::init();
    }
};

template<>
struct Devices<SDR01, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 48'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 
//    using systemTimer = STM32::SystemTimer<clock, UseInterrupts<true>, MCU>; 
//    using trace = Arm::Trace<clock>; // 2MHz ITM Takt
    using trace = Arm::Trace<clock, 24_MHz>; 
    
    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    using pina0 = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using pina1 = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using pina2 = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using pina3 = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pina8 = Mcu::Stm::Pin<gpioa, 8, MCU>;
    using sda = pina8;
    using pina9 = Mcu::Stm::Pin<gpioa, 9, MCU>;
    using scl = pina9;
    using pina11 = Mcu::Stm::Pin<gpioa, 11, MCU>;
    
    using pinb0 = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using pinb4 = Mcu::Stm::Pin<gpiob, 4, MCU>; // changed to pa11, now af2
    using pinb5 = Mcu::Stm::Pin<gpiob, 5, MCU>;
    using pinb6 = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using led = Mcu::Stm::Pin<gpiob, 8, MCU>;

    using pinf0 = Mcu::Stm::Pin<gpiof, 0, MCU>;
    using pinf1 = Mcu::Stm::Pin<gpiof, 1, MCU>;
    
    using dac1 = Mcu::Stm::Dac<1, MCU>;
    using dac3 = Mcu::Stm::Dac<3, MCU>;
    
//    using adc1 = Mcu::Stm::Adc<1, 13, MCU>; // opamp1 out
    using adc2 = Mcu::Stm::Adc<2, 16, Mcu::Stm::NoTriggerSource, MCU>; // opamp2 out

//    using pga1 = Mcu::Stm::PGA<1, MCU>;
    using pga2 = Mcu::Stm::PGA<2, MCU>;
    
    using follow1 = Mcu::Stm::Follower<1, MCU>;
    
    using period = std::integral_constant<uint16_t, 5120>;
    using prescaler = std::integral_constant<uint16_t, 664>;
    
    using pwm1 = Mcu::Stm::Timer<3, period, prescaler, Mcu::Stm::Trigger<false>, MCU>;
//    using pina6 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    using pwm2 = Mcu::Stm::Timer<4, period, prescaler, Mcu::Stm::Trigger<false>, MCU>;
    
    using i2c2 = Mcu::Stm::I2C::Master<2, 16, MCU>;
    static inline constexpr Mcu::Stm::I2C::Address si5351{0x60};
    using si = External::SI5351::Clock<i2c2, si5351>;
    
    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();
        
        gpioa::init();
        gpiob::init();
        gpiof::init();

        pinb0::template dir<Mcu::Output>();
        pinb0::template speed<Mcu::High>();
        pinb5::template dir<Mcu::Output>();
        pinb5::template speed<Mcu::High>();
        pinb6::template dir<Mcu::Output>();
        pinb6::template speed<Mcu::High>();
        led::template dir<Mcu::Output>();

        pina11::template dir<Mcu::Output>();
        pina11::template speed<Mcu::High>();
        
//        pina0::analog();
//        pina1::analog();
//        pina2::analog();
//        pina3::analog();

//        pga1::init();
//        pga1::gain(0x0a);
        
        pga2::init();
        pga2::gain(0x08); // x2
        pga2::gain(0x09); // x4
//        pga2::gain(0x0a); // x8
//        pga2::gain(0x0b); // x16
//        pga2::gain(0x0c); // x32
        
        follow1::init();
        
        dac1::init();
        dac3::init();
        
//        adc1::init();
        adc2::init();
        
//        pinb4::template dir<Mcu::Output>();
//        pinb4::template speed<Mcu::High>();
        pinb4::afunction(2);
        pwm1::init();
        
        sda::openDrain();
        scl::openDrain();
        sda::afunction(4);
        scl::afunction(4);
        
        i2c2::init();
    }
};

template<>
struct Devices<FSK01, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 48'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 
//    using systemTimer = STM32::SystemTimer<clock, UseInterrupts<true>, MCU>; 
    using trace = Arm::Trace<clock>;
    
    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    using pina0 = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using pina1 = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using pina2 = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using pina3 = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pina11 = Mcu::Stm::Pin<gpioa, 11, MCU>;
    
    using pinb0 = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using pinb4 = Mcu::Stm::Pin<gpiob, 4, MCU>; // changed to pa11, now af2
    using pinb5 = Mcu::Stm::Pin<gpiob, 5, MCU>;
    using pinb6 = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using led = Mcu::Stm::Pin<gpiob, 8, MCU>;

    using pinf0 = Mcu::Stm::Pin<gpiof, 0, MCU>;
    using pinf1 = Mcu::Stm::Pin<gpiof, 1, MCU>;
    
    using dac1 = Mcu::Stm::Dac<1, MCU>;
    using dac3 = Mcu::Stm::Dac<3, MCU>;
    
//    using adc1 = Mcu::Stm::Adc<1, 13, MCU>; // opamp1 out
    using adc2 = Mcu::Stm::Adc<2, 16, Mcu::Stm::NoTriggerSource, MCU>; // opamp2 out

//    using pga1 = Mcu::Stm::PGA<1, MCU>;
    using pga2 = Mcu::Stm::PGA<2, MCU>;
    
    using follow1 = Mcu::Stm::Follower<1, MCU>;
    
    using period = std::integral_constant<uint16_t, 5120>;
    using prescaler = std::integral_constant<uint16_t, 664>;
    
    using pwm1 = Mcu::Stm::Timer<3, period, prescaler, Mcu::Stm::Trigger<false>, MCU>;
//    using pina6 = Mcu::Stm::Pin<gpioa, 6, MCU>;
    
    using pwm2 = Mcu::Stm::Timer<4, period, prescaler, Mcu::Stm::Trigger<false>, MCU>;
    
    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();
        
        gpioa::init();
        gpiob::init();
        gpiof::init();

        pinb0::template dir<Mcu::Output>();
        pinb0::template speed<Mcu::High>();
        pinb5::template dir<Mcu::Output>();
        pinb5::template speed<Mcu::High>();
        pinb6::template dir<Mcu::Output>();
        pinb6::template speed<Mcu::High>();
        led::template dir<Mcu::Output>();

        pina11::template dir<Mcu::Output>();
        pina11::template speed<Mcu::High>();
        
//        pina0::analog();
//        pina1::analog();
//        pina2::analog();
//        pina3::analog();

//        pga1::init();
//        pga1::gain(0x0a);
        
        pga2::init();
        pga2::gain(0x08); // x2
        pga2::gain(0x09); // x4
//        pga2::gain(0x0a); // x8
//        pga2::gain(0x0b); // x16
//        pga2::gain(0x0c); // x32
        
        follow1::init();
        
        dac1::init();
        dac3::init();
        
//        adc1::init();
        adc2::init();
        
//        pinb4::template dir<Mcu::Output>();
//        pinb4::template speed<Mcu::High>();
        pinb4::afunction(2);
        pwm1::init();
    }
};

template<>
struct Devices<FSK, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 48'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 
//    using systemTimer = STM32::SystemTimer<clock, UseInterrupts<true>, MCU>; 
    using trace = Arm::Trace<clock>;
    
    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    using pina0 = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using pina1 = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using pina2 = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using pina3 = Mcu::Stm::Pin<gpioa, 3, MCU>;
    
    using pinb0 = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using pinb4 = Mcu::Stm::Pin<gpiob, 4, MCU>;
    using pinb5 = Mcu::Stm::Pin<gpiob, 5, MCU>;
    using pinb6 = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using led = Mcu::Stm::Pin<gpiob, 8, MCU>;

    using pinf0 = Mcu::Stm::Pin<gpiof, 0, MCU>;
    using pinf1 = Mcu::Stm::Pin<gpiof, 1, MCU>;
    
    using dac1 = Mcu::Stm::Dac<1, MCU>;
    using dac3 = Mcu::Stm::Dac<3, MCU>;
    
//    using adc1 = Mcu::Stm::Adc<1, 13, MCU>; // opamp1 out
    using adc2 = Mcu::Stm::Adc<2, 16, Mcu::Stm::NoTriggerSource, MCU>; // opamp2 out

//    using pga1 = Mcu::Stm::PGA<1, MCU>;
    using pga2 = Mcu::Stm::PGA<2, MCU>;
    
    using follow1 = Mcu::Stm::Follower<1, MCU>;
    
    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();
        
        gpioa::init();
        gpiob::init();
        gpiof::init();

        pinb0::template dir<Mcu::Output>();
        pinb0::template speed<Mcu::High>();
        pinb4::template dir<Mcu::Output>();
        pinb4::template speed<Mcu::High>();
        pinb5::template dir<Mcu::Output>();
        pinb5::template speed<Mcu::High>();
        pinb6::template dir<Mcu::Output>();
        pinb6::template speed<Mcu::High>();
        led::template dir<Mcu::Output>();

//        pina0::analog();
//        pina1::analog();
//        pina2::analog();
//        pina3::analog();

//        pga1::init();
//        pga1::gain(0x0a);
        
        pga2::init();
        pga2::gain(0x08); // x2
        pga2::gain(0x09); // x4
//        pga2::gain(0x0a); // x8
//        pga2::gain(0x0b); // x16
//        pga2::gain(0x0c); // x32
        
        follow1::init();
        
        dac1::init();
        dac3::init();
        
//        adc1::init();
        adc2::init();
    }
};

template<>
struct Devices<Test01, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 48'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 
//    using systemTimer = STM32::SystemTimer<clock, UseInterrupts<true>, MCU>; 
    using trace = Arm::Trace<clock>;
    
    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    using pina0 = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using pina1 = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using pina2 = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using pina3 = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using pina5 = Mcu::Stm::Pin<gpioa, 5, MCU>;
    using pina7 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    
//    using pinb0 = Mcu::Stm::Pin<gpiob, 0, MCU>;
    using pinb4 = Mcu::Stm::Pin<gpiob, 4, MCU>;
//    using pinb5 = Mcu::Stm::Pin<gpiob, 5, MCU>;
//    using pinb8 = Mcu::Stm::Pin<gpiob, 8, MCU>;

//    using pinf0 = Mcu::Stm::Pin<gpiof, 0, MCU>;
//    using pinf1 = Mcu::Stm::Pin<gpiof, 1, MCU>;
    
    using dac1 = Mcu::Stm::Dac<1, MCU>;
    using dac3 = Mcu::Stm::Dac<3, MCU>;
    
//    using adc1 = Mcu::Stm::Adc<1, 13, MCU>; // opamp1 out
//    using adc2 = Mcu::Stm::Adc<2, 16, MCU>; // opamp2 out

//    using pga1 = Mcu::Stm::PGA<1, MCU>;
//    using pga2 = Mcu::Stm::PGA<2, MCU>;
    
    using follow1 = Mcu::Stm::Follower<1, MCU>;
    
    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();
        
        gpioa::init();
        gpiob::init();
        gpiof::init();

//        pinb0::template dir<Mcu::Output>();
//        pinb0::template speed<Mcu::High>();
        pinb4::template dir<Mcu::Output>();
        pinb4::template speed<Mcu::High>();
//        pinb5::template dir<Mcu::Output>();
//        pinb5::template speed<Mcu::High>();
//        pinb8::template dir<Mcu::Output>();

//        pina2::template dir<Mcu::Output>();
//        pina0::analog();
//        pina1::analog();
//        pina2::analog();
//        pina3::analog();
//        pina5::analog();
//        pina7::analog();

        dac1::init();
        dac3::init();
        
        follow1::init();
        
    }
};

template<typename HW>
struct Devices<HW, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 1000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 
//    using systemTimer = STM32::SystemTimer<clock, UseInterrupts<true>, MCU>; 
    using trace = Arm::Trace<clock>;
    
    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    using led = Mcu::Stm::Pin<gpiob, 8, MCU>;
    using pinb4 = Mcu::Stm::Pin<gpiob, 4, MCU>;

    using pinf0 = Mcu::Stm::Pin<gpiof, 0, MCU>;
    using pinf1 = Mcu::Stm::Pin<gpiof, 1, MCU>;

    using pina0 = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using pina1 = Mcu::Stm::Pin<gpioa, 1, MCU>;
    
    using dac1 = Mcu::Stm::Dac<1, MCU>;
    using adc1 = Mcu::Stm::Adc<1, 1, MCU>;
    using adc2 = Mcu::Stm::Adc<2, 2, MCU>;
    
    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();
        
//        gpioa::init();
        gpiob::init();
//        gpiof::init();
         
        led::template dir<Mcu::Output>();
        pinb4::template dir<Mcu::Output>();
        pinb4::template speed<Mcu::High>();
        
//        pinf0::analog();
//        pinf1::analog();

//        pina0::analog();
//        pina1::analog();
        
        dac1::init();
        adc1::init();
    }
};

