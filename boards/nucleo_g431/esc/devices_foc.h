#pragma once

#include "../include/mcu.h"
#include "../include/dma.h"
#include "../include/exti.h"
//#include "../include/bldc.h"
#include "../include/foc.h"
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

struct Storage {
    static inline std::array<uint16_t, 4> adcValues{};
};


struct ESC01;
struct ESC_HALL_01;
struct ESC_HALL_02;
struct ESC_FOC_01;

template<typename HW, typename Config, typename MCU = void>
struct Devices;

using namespace Mcu::Stm;

template<typename Config>
struct Devices<ESC_FOC_01, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 1'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 

    // Timer 6
//    using adc_period = std::integral_constant<uint16_t, 170'000'000 / 10'000>;
//    using adc_prescaler = std::integral_constant<uint16_t, 0>; // pre=1
//    using adc_timer = Mcu::Stm::Timer<6, adc_period, adc_prescaler, Mcu::Stm::Trigger<true>, MCU>;
    
    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    using pintx = Mcu::Stm::Pin<gpiob, 3, MCU>;
    using pinrx = Mcu::Stm::Pin<gpiob, 4, MCU>;

    using serial2 = Mcu::Stm::Uart<2, void, 64, char, clock, MCU>;
    
    using led = Mcu::Stm::Pin<gpioc, 6, MCU>;
    using button = Mcu::Stm::Pin<gpioc, 10, MCU>;
    
    using tp2 = Mcu::Stm::Pin<gpioc, 11, MCU>;
    using tp3 = Mcu::Stm::Pin<gpiob, 1, MCU>; // opamp3_out

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

    // Timer 6
    using measure_prescaler = std::integral_constant<uint16_t, 169>; // 1 MHz
    using measurement = Mcu::Stm::Motor::Measurement<6, measure_prescaler, driver, MCU>;
        
    using hall1 = Mcu::Stm::Pin<gpiob, 6, MCU>; 
    using hall2 = Mcu::Stm::Pin<gpiob, 7, MCU>; 
    using hall3 = Mcu::Stm::Pin<gpiob, 8, MCU>; 

//    using dac3 = Mcu::Stm::Dac<3, MCU>; 
//    using follow3 = Mcu::Stm::Follower<3, MCU>; // tp3

    using pga1 = Mcu::Stm::PGA<1>; // PA1, PA3
    using pga2 = Mcu::Stm::PGA<2>; // PA5, PA7
    using pga3 = Mcu::Stm::PGA<3>; // PB0, PB2
    
//    using pos_prescaler = std::integral_constant<uint16_t, 169>; // 1Mhz
    using pos_prescaler = std::integral_constant<uint16_t, 339>; 
    // Timer 7 
    using pos_estimator = Mcu::Stm::Motor::Estimator<7, pos_prescaler, driver, void, MCU>; 
    
//    using hall_prescaler = std::integral_constant<uint16_t, 169>; // 1MHz
//    using hall_prescaler = std::integral_constant<uint16_t, 339>;
    // Timer 4
    using hall = Mcu::Stm::Hall<4, pos_prescaler, driver, pos_estimator, measurement, MCU>;
    
//    using adc1 = Mcu::Stm::Adc<1, 11, driver, MCU>; // Potentiometer
//    using adc1 = Mcu::Stm::Adc<1, 11, adc_timer, Mcu::Stm::UseDma<true>, MCU>; // Potentiometer
    using adc1 = Mcu::Stm::Adc<1, 11, driver, Mcu::Stm::UseDma<true>, MCU>; // Potentiometer
    
//    using dma1 = Mcu::Stm::Dma<1, 0, MCU>;
//    using dma1Channel0 = Mcu::Stm::dma1::Channel<0, Storage::adcValues>;
//    using dmamux = Mcu::Stm::DmaMux<adc1, dma1Channel0>;    
    
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
        serial2::baud(115200);
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

        measurement::init();
        
//        dac3::init();
//        follow3::init();
        
        // config dma
        {
            RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
            RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
            
            MODIFY_REG(DMA1_Channel1->CCR, DMA_CCR_MSIZE_Msk, 0x01 << DMA_CCR_MSIZE_Pos);
            MODIFY_REG(DMA1_Channel1->CCR, DMA_CCR_PSIZE_Msk, 0x01 << DMA_CCR_PSIZE_Pos);
            DMA1_Channel1->CCR |= DMA_CCR_MINC;
            DMA1_Channel1->CCR |= DMA_CCR_CIRC;
            DMA1_Channel1->CCR &= ~DMA_CCR_DIR;
            DMA1_Channel1->CNDTR = 2;
            DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
            DMA1_Channel1->CMAR = (uint32_t)&Storage::adcValues[0];
            DMA1_Channel1->CCR |= DMA_CCR_TCIE; // for test
            DMA1_Channel1->CCR |= DMA_CCR_EN;
            
            DMAMUX1_Channel0->CCR = 5 & DMAMUX_CxCR_DMAREQ_ID_Msk; // ADC1
            DMAMUX1_Channel0->CCR |= DMAMUX_CxCR_EGE;
            
            MODIFY_REG(DMAMUX1_RequestGenerator0->RGCR, DMAMUX_RGxCR_GPOL_Msk, 0b01 << DMAMUX_RGxCR_GPOL_Pos);
            DMAMUX1_RequestGenerator0->RGCR |= DMAMUX_RGxCR_GE;
        }
        
        pga1::init();
        pga2::init();
        pga3::init();
    }
};
