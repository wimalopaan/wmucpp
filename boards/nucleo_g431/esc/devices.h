#pragma once

#include "../include/mcu.h"
#include "../include/exti.h"
#include "../include/bldc.h"
#include "../include/pwm.h"
#include "../include/rc.h"
#include "../include/usart.h"
#include "../include/si5351.h"
#include "../include/i2c.h"
#include "../include/timer.h"
#include "../include/opamp.h"
#include "../include/adc.h"
#include "../include/dac.h"
#include "../include/clock.h"
#include "../include/units.h"
#include "../include/output.h"
#include "../include/concepts.h"
#include "../include/mcu_traits.h"
#include "../include/arm.h"
#include "../include/gpio.h"
#include "../include/tick.h"
#include "../include/meta.h"
 
#include "driver.h"

struct ESC01;
struct ESC_HALL_01;
struct ESC_HALL_02;

template<typename HW, typename Config, typename MCU = void>
struct Devices;

using namespace Mcu::Stm;

template<typename Config>
struct Devices<ESC_HALL_02, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 1'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 

    using adc_period = std::integral_constant<uint16_t, 170'000'000 / 48'000>;
    using adc_prescaler = std::integral_constant<uint16_t, 0>; // pre=1
    using adc_timer = Mcu::Stm::Timer<6, adc_period, adc_prescaler, Mcu::Stm::Trigger<true>, MCU>;
    
    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using pintx = Mcu::Stm::Pin<gpiob, 3, MCU>;
    using pinrx = Mcu::Stm::Pin<gpiob, 4, MCU>;

    using serial2 = Mcu::Stm::Uart<2, RC::Protokoll::Null::Adapter<>, 64, char, clock, MCU>;
    
    using led = Mcu::Stm::Pin<gpioc, 6, MCU>;
    using button = Mcu::Stm::Pin<gpioc, 10, MCU>;
    
    using tp2 = Mcu::Stm::Pin<gpioc, 11, MCU>;
    using tp3 = Mcu::Stm::Pin<gpiob, 1, MCU>;

    using phaseUH = Mcu::Stm::Pin<gpioa, 8, MCU>; // af6 tim1ch1
    using phaseUL = Mcu::Stm::Pin<gpioc, 13, MCU>; // af4 tim1ch1n
    using phaseU  = Mcu::Stm::Motor::Phase<Motor::Switch<phaseUH, AlternateFunctions::CC<1>>, Motor::Switch<phaseUL, AlternateFunctions::CC<1, AlternateFunctions::Negativ>>>;
    
    using phaseVH = Mcu::Stm::Pin<gpioa, 9, MCU>; // af6 tim1ch2
    using phaseVL = Mcu::Stm::Pin<gpioa, 12, MCU>; // af6 tim1ch3n
    using phaseV  = Mcu::Stm::Motor::Phase<Motor::Switch<phaseVH, AlternateFunctions::CC<2>>, Motor::Switch<phaseVL, AlternateFunctions::CC<2, AlternateFunctions::Negativ>>>;

    using phaseWH = Mcu::Stm::Pin<gpioa, 10, MCU>; // af6 tim1ch3
    using phaseWL = Mcu::Stm::Pin<gpiob, 15, MCU>; // af4 tim1ch3n
    using phaseW  = Mcu::Stm::Motor::Phase<Motor::Switch<phaseWH, AlternateFunctions::CC<3>>, Motor::Switch<phaseWL, AlternateFunctions::CC<3, AlternateFunctions::Negativ>>>;

    using pwm_period = std::integral_constant<uint16_t, (170'000'000 / 20'000)>; 
    
    using driver = Mcu::Stm::Motor::Driver<1, phaseU, phaseV, phaseW, pwm_period, clock>;
    
    using hall1 = Mcu::Stm::Pin<gpiob, 6, MCU>; 
    using hall2 = Mcu::Stm::Pin<gpiob, 7, MCU>; 
    using hall3 = Mcu::Stm::Pin<gpiob, 8, MCU>; 

    using dac3 = Mcu::Stm::Dac<3, MCU>;
    using follow3 = Mcu::Stm::Follower<3, MCU>;
        
    using pos_prescaler = std::integral_constant<uint16_t, 169>; // 1Mhz
    using pos_estimator = Mcu::Stm::Motor::Estimator<7, pos_prescaler, driver, dac3, MCU>;
    
    using hall_prescaler = std::integral_constant<uint16_t, 169>; // 1MHz
    using hall = Mcu::Stm::Hall<4, hall_prescaler, driver, pos_estimator, MCU>;
    
//    using adc1 = Mcu::Stm::Adc<1, 11, driver, MCU>; // Potentiometer
    using adc1 = Mcu::Stm::Adc<1, 11, adc_timer, MCU>; // Potentiometer
    
    static inline void init() {
        clock::init();
        systemTimer::init();
        adc_timer::init();
        
        gpioa::init();
        gpiob::init();
        gpioc::init();

        led::template dir<Mcu::Output>();
        button::template dir<Mcu::Input>();
        
        tp2::template dir<Mcu::Output>();
//        tp3::template dir<Mcu::Output>();
       
        serial2::init();
        serial2::baud(9600);
        pintx::afunction(7);
        pinrx::afunction(7);
        
        adc1::init();
        
        driver::init();
        
        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS); // solves the problem with hall1 input: https://community.simplefoc.com/t/b-g431b-esc1-problem-with-hall-sensor/1875/20

        hall::init();
        hall1::afunction(2);
        hall2::afunction(2);
        hall3::afunction(2);
        
        pos_estimator::init();
        
        dac3::init();
        follow3::init();
    }
};


template<typename Config>
struct Devices<ESC_HALL_01, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 1'00_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 

    using adc_period = std::integral_constant<uint16_t, 170'000'000 / 48'000>;
    using adc_prescaler = std::integral_constant<uint16_t, 0>; // pre=1
    using adc_timer = Mcu::Stm::Timer<6, adc_period, adc_prescaler, Mcu::Stm::Trigger<true>, MCU>;
    
    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using pintx = Mcu::Stm::Pin<gpiob, 3, MCU>;
    using pinrx = Mcu::Stm::Pin<gpiob, 4, MCU>;

    using serial2 = Mcu::Stm::Uart<2, RC::Protokoll::Null::Adapter<>, 64, char, clock, MCU>;
    
    using led = Mcu::Stm::Pin<gpioc, 6, MCU>;
    using button = Mcu::Stm::Pin<gpioc, 10, MCU>;
    
    using tp2 = Mcu::Stm::Pin<gpioc, 11, MCU>;
//    using tp3 = Mcu::Stm::Pin<gpiob, 1, MCU>;

    using phaseUH = Mcu::Stm::Pin<gpioa, 8, MCU>; // af6 tim1ch1
    using phaseUL = Mcu::Stm::Pin<gpioc, 13, MCU>; // af4 tim1ch1n
    using phaseU  = Mcu::Stm::Motor::Phase<Motor::Switch<phaseUH, AlternateFunctions::CC<1>>, Motor::Switch<phaseUL, AlternateFunctions::CC<1, AlternateFunctions::Negativ>>>;
    
    using phaseVH = Mcu::Stm::Pin<gpioa, 9, MCU>; // af6 tim1ch2
    using phaseVL = Mcu::Stm::Pin<gpioa, 12, MCU>; // af6 tim1ch3n
    using phaseV  = Mcu::Stm::Motor::Phase<Motor::Switch<phaseVH, AlternateFunctions::CC<2>>, Motor::Switch<phaseVL, AlternateFunctions::CC<2, AlternateFunctions::Negativ>>>;

    using phaseWH = Mcu::Stm::Pin<gpioa, 10, MCU>; // af6 tim1ch3
    using phaseWL = Mcu::Stm::Pin<gpiob, 15, MCU>; // af4 tim1ch3n
    using phaseW  = Mcu::Stm::Motor::Phase<Motor::Switch<phaseWH, AlternateFunctions::CC<3>>, Motor::Switch<phaseWL, AlternateFunctions::CC<3, AlternateFunctions::Negativ>>>;

    using pwm_period = std::integral_constant<uint16_t, (170'000'000 / 20'000)>; 
    
    using driver = Mcu::Stm::Motor::Driver<1, phaseU, phaseV, phaseW, pwm_period, clock>;
    
    using hall1 = Mcu::Stm::Pin<gpiob, 6, MCU>; 
    using hall2 = Mcu::Stm::Pin<gpiob, 7, MCU>; 
    using hall3 = Mcu::Stm::Pin<gpiob, 8, MCU>; 
    
    using hall_prescaler = std::integral_constant<uint16_t, 169>;    
    using hall = Mcu::Stm::Hall<4, hall_prescaler, driver, MCU>;
    
//    using exti = Mcu::Stm::ExtInt<hall1, MCU>;
    
//    using adc1 = Mcu::Stm::Adc<1, 11, driver, MCU>; // Potentiometer
    using adc1 = Mcu::Stm::Adc<1, 11, adc_timer, MCU>; // Potentiometer
    
    static inline void init() {
        clock::init();
        systemTimer::init();
        adc_timer::init();
        
        gpioa::init();
        gpiob::init();
        gpioc::init();

        led::template dir<Mcu::Output>();
        button::template dir<Mcu::Input>();
        
        tp2::template dir<Mcu::Output>();
//        tp3::template dir<Mcu::Output>();
       
        serial2::init();
        serial2::baud(9600);
        pintx::afunction(7);
        pinrx::afunction(7);
        
        adc1::init();
        
        driver::init();
        
        phaseUH::template dir<Mcu::Output>();
        phaseUL::template dir<Mcu::Output>();
        phaseVH::template dir<Mcu::Output>();
        phaseVL::template dir<Mcu::Output>();
        phaseWH::template dir<Mcu::Output>();
        phaseWL::template dir<Mcu::Output>();
        
        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS); // solves the problem with hall1 input: https://community.simplefoc.com/t/b-g431b-esc1-problem-with-hall-sensor/1875/20

        hall::init();
        hall1::afunction(2);
        hall2::afunction(2);
        hall3::afunction(2);
        
//        exti::init();
    }
};

template<typename Config>
struct Devices<ESC01, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 

//    using adc_period = std::integral_constant<uint16_t, 170'000'000 / 48'000>;
//    using adc_prescaler = std::integral_constant<uint16_t, 0>; // pre=1
//    using adc_timer = Mcu::Stm::Timer<6, adc_period, adc_prescaler, Mcu::Stm::Trigger<true>, MCU>;
    
    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using pintx = Mcu::Stm::Pin<gpiob, 3, MCU>;
    using pinrx = Mcu::Stm::Pin<gpiob, 4, MCU>;

    using serial2 = Mcu::Stm::Uart<2, RC::Protokoll::Null::Adapter<>, 64, char, clock, MCU>;
    
    using led = Mcu::Stm::Pin<gpioc, 6, MCU>;
    using button = Mcu::Stm::Pin<gpioc, 10, MCU>;
    
    using tp2 = Mcu::Stm::Pin<gpioc, 11, MCU>;
    using tp3 = Mcu::Stm::Pin<gpiob, 1, MCU>;

    using phaseUH = Mcu::Stm::Pin<gpioa, 8, MCU>; // af6 tim1ch1
    using phaseUL = Mcu::Stm::Pin<gpioc, 13, MCU>; // af4 tim1ch1n

    using phaseVH = Mcu::Stm::Pin<gpioa, 9, MCU>; // af6 tim1ch2
    using phaseVL = Mcu::Stm::Pin<gpioa, 12, MCU>; // af6 tim1ch3n

    using phaseWH = Mcu::Stm::Pin<gpioa, 10, MCU>; // af6 tim1ch3
    using phaseWL = Mcu::Stm::Pin<gpiob, 15, MCU>; // af4 tim1ch3n

    using pwm_period = std::integral_constant<uint16_t, (170'000'000 / 18'000)>; 
    using pwm_prescaler = std::integral_constant<uint16_t, 0>;
    
    using pwm = Mcu::Stm::Bldc<1, pwm_period, pwm_prescaler>;
    
    using driver = Sinus::Driver<pwm>;

    using hall1 = Mcu::Stm::Pin<gpiob, 6, MCU>; 
    using hall2 = Mcu::Stm::Pin<gpiob, 7, MCU>; 
    using hall3 = Mcu::Stm::Pin<gpiob, 8, MCU>; 
    
//    using hall_prescaler = std::integral_constant<uint16_t, (170'000'000 / 170'000'000)>;    
    using hall_prescaler = std::integral_constant<uint16_t, 0>;    
    using hall = Mcu::Stm::Hall<4, hall_prescaler, driver, MCU>;
    
    using adc1 = Mcu::Stm::Adc<1, 11, pwm, MCU>; // Potentiometer
    
    static inline void init() {
        clock::init();
        systemTimer::init();
//        adc_timer::init();
        
        gpioa::init();
        gpiob::init();
        gpioc::init();

        led::template dir<Mcu::Output>();
        button::template dir<Mcu::Input>();
        
        tp2::template dir<Mcu::Output>();
        tp3::template dir<Mcu::Output>();
       
        serial2::init();
        serial2::baud(9600);
        pintx::afunction(7);
        pinrx::afunction(7);
        
        adc1::init();
        
        driver::init();
        
        phaseUH::template dir<Mcu::Output>();
        phaseUL::template dir<Mcu::Output>();
        phaseUH::afunction(6);
        phaseUL::afunction(4);

        phaseVH::template dir<Mcu::Output>();
        phaseVL::template dir<Mcu::Output>();
        phaseVH::afunction(6);
        phaseVL::afunction(6);

        phaseWH::template dir<Mcu::Output>();
        phaseWL::template dir<Mcu::Output>();
        phaseWH::afunction(6);
        phaseWL::afunction(4);
        
        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS); // solves the problem with hall1 input: https://community.simplefoc.com/t/b-g431b-esc1-problem-with-hall-sensor/1875/20

        hall::init();
        hall1::afunction(2);
        hall2::afunction(2);
        hall3::afunction(2);
    }
};

