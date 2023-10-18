#pragma once

#include "../include/mcu.h"
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
 
struct RC01;

template<typename HW, typename Config, typename MCU = void>
struct Devices;

using namespace Mcu::Stm;

struct TestProvider {
    inline static constexpr auto valueId = RC::Protokoll::SPort::ValueId::DIY;
    inline static constexpr auto ibus_type = RC::Protokoll::IBus::Type::type::ANGLE;
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
        return ++mValue;
    }
    inline static uint16_t mValue{};
};

template<typename Config>
struct Devices<RC01, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
//    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 1'000_Hz, Mcu::Stm::HSI>, MCU>;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 
    using trace = Arm::Trace<clock, 24_MHz, 128>; 

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;

//    using servotx = Mcu::Stm::Pin<gpiob, 6, MCU>;
//    using servorx = Mcu::Stm::Pin<gpiob, 7, MCU>;
//    using sporttx = Mcu::Stm::Pin<gpioa, 9, MCU>;
//    using sportrx = Mcu::Stm::Pin<gpiob, 7, MCU>;

    using ibusrxtx = Mcu::Stm::Pin<gpiob, 6, MCU>;
    
//    using servo_pa = RC::Protokoll::IBus::Adapter<0, void>;
//    using servo_pa = RC::Protokoll::SBus::Servo::Adapter<0, systemTimer, void>;
//    using servo = Mcu::Stm::Uart<1, servo_pa, 0, char, clock, MCU>;
    
//    template<typename PA>
//    using sport_uart = Mcu::Stm::Uart<1, PA, 16, std::byte, clock, MCU>;
//    using sport_sensor = RC::Protokoll::SPort::Sensor<RC::Protokoll::SPort::SensorId::ID3, sport_uart, systemTimer, Meta::List<TestProvider>>;

    template<typename PA>
    using ibus_uart = Mcu::Stm::Uart<1, PA, 16, std::byte, clock, MCU>;
    using ibus_sensor = RC::Protokoll::IBus::Sensor<ibus_uart, systemTimer, Meta::List<TestProvider>>;
    
    using led = Mcu::Stm::Pin<gpiob, 8, MCU>;

    using pinb4 = Mcu::Stm::Pin<gpiob, 4, MCU>; 
    using pinb5 = Mcu::Stm::Pin<gpiob, 5, MCU>;
    using cppm_period = std::integral_constant<uint16_t, 5120>; // duty 512 -> 2ms, duty 256 -> 1ms Pulse, Period = 20ms
    using cppm_prescaler = std::integral_constant<uint16_t, 664>;
    using timerPwm1 = Mcu::Stm::Timer<3, cppm_period, cppm_prescaler, Mcu::Stm::Trigger<false>, MCU>;
    using pwm1 = Mcu::Stm::Pwm::Servo<timerPwm1, Meta::List<pinb4, void, pinb5, void>>; // bis zu 4 Ausgänge
    
    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();
        
        gpioa::init();
        gpiob::init();

//        servotx::afunction(7);
//        servorx::afunction(7);
//        servorx::pullup<true>();
//        servorx::pulldown<true>();
        
//        sporttx::afunction(7);
//        sportrx::afunction(7);

        ibusrxtx::afunction(7);
        ibusrxtx::pullup();
        ibusrxtx::openDrain();
        
        //        servo::init();
//        servo::parity(true);
//        servo::stop(2);
//        servo::invert(true);
//        servo::baud(100'000);

//        sport_sensor::init();
//        sport_sensor::uart::init();
//        sport_sensor::uart::baud(57600);
//        sport_sensor::uart::invert(true);
        
        ibus_sensor::init();
        
        led::template dir<Mcu::Output>();
       
        pwm1::init();
        
    }
};

