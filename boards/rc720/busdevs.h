#pragma once

#include <mcu/avr.h>

#include <mcu/internals/usart.h>

#include <external/sbus/sbus.h>
#include <external/ibus/ibus2.h>

#include "version.h"

template<typename Bus>
struct BusDevs;

template<typename Devs>
struct BusDevs<External::Bus::IBusIBus<Devs>> {
    using bus_type = External::Bus::IBusIBus<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
    using servo = AVR::Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

#ifdef LOG_OUTPUT
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif
    
    using adcController = devs::adcController;

    using value_t = servo_pa::value_type;
    using pwm0 = PPMAdapter<typename devs::pwm0_2, 1>;
    using adc0 = ADCAdapter<typename devs::adcController, 1>;
    using servo_0 = External::Servo<pwm0, adc0, systemTimer, value_t, typename terminal::device_type>;
    using pwm2 = PPMAdapter<typename devs::pwm0_2, 0>;
    using esc0 = External::EscFsm<pwm2, systemTimer, value_t, servo_0>;
    
    using pwm1 = PPMAdapter<typename devs::pwm0_2, 2>;
    using adc1 = ADCAdapter<typename devs::adcController, 0>;
    using servo_1 = External::Servo<pwm1, adc1, systemTimer, value_t, typename terminal::device_type>;
    using pwm3 = PPMAsyncAdapter<typename devs::pwm3>;
    using esc1 = External::EscFsm<pwm3, systemTimer, value_t, servo_0>;

    using pwm4 = PPMAsyncAdapter<typename devs::pwm4>;
    using pwm5 = PPMAsyncAdapter<typename devs::pwm5>;
    
    using tempiP = External::InternalTempProvider<typename Devs::adcController, 2, typename Devs::sigrow, bus_type>;

    using s0pp = servo_0::PositionProvider;
    using s1pp = servo_1::PositionProvider;
    
    using sensor = IBus2::Sensor<typename Devs::sensorPosition, AVR::Usart, AVR::BaudRate<115200>, 
                                Meta::List<tempiP, VersionProvider, s0pp, s1pp>, 
                                systemTimer, typename devs::ibt>;

    inline static void init(const bool) {
        devs::eeprom::init();
        if (!((devs::eeprom::data().magic() == 42))) {
            devs::eeprom::data().clear();
            devs::eeprom::data().change();
        }            
        
        devs::lut2::init(std::byte{0x00}); // low on lut3-out 
        sensor::init();
        sensor::uart::txOpenDrain();
        
        servo::template init<AVR::BaudRate<115200>>();
        etl::outl<terminal>("IB"_pgm);

        devs::adcController::init();
        devs::adc::nsamples(4);
        
        devs::pwm0_2::init();
        devs::pwm3::init();
//        devs::pwm4::init(); // later initialised
//        devs::pwm5::init();
        
        servo_0::init();
        servo_1::init();
        
        esc0::init();
        esc1::init();
    }
    static inline void onPulse() {
        devs::pwm3::onReload([]{
            devs::evrouter::template strobe<0>();
        });
        devs::pwm4::onReload([]{
            devs::evrouter::template strobe<1>();
        });
        devs::pwm5::onReload([]{
            devs::evrouter::template strobe<2>();
        });
    }
    
};

template<typename Devs>
struct BusDevs<External::Bus::SBusSPort<Devs>> {
    using bus_type = External::Bus::SBusSPort<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using servo = AVR::Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    
#ifdef LOG_OUTPUT
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif
    
    template<typename PA>
    using sensorUsart = AVR::Usart<typename Devs::sensorPosition, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<64>>;
    
    using adcController = devs::adcController;
    
    using value_t = servo_pa::value_type;
    using pwm0 = PPMAdapter<typename devs::pwm0_2, 1>;
    using adc0 = ADCAdapter<typename devs::adcController, 1>;
    using servo_0 = External::Servo<pwm0, adc0, systemTimer, value_t, typename terminal::device_type>;
    using pwm2 = PPMAdapter<typename devs::pwm0_2, 0>;
    using esc0 = External::EscFsm<pwm2, systemTimer, value_t, servo_0>;
    
    using pwm1 = PPMAdapter<typename devs::pwm0_2, 2>;
    using adc1 = ADCAdapter<typename devs::adcController, 0>;
    using servo_1 = External::Servo<pwm1, adc1, systemTimer, value_t, typename terminal::device_type>;
    using pwm3 = PPMAsyncAdapter<typename devs::pwm3>;
    using esc1 = External::EscFsm<pwm3, systemTimer, value_t, servo_0>;
    
    using pwm4 = PPMAsyncAdapter<typename devs::pwm4>;
    using pwm5 = PPMAsyncAdapter<typename devs::pwm5>;

    using tempiP = External::InternalTempProvider<typename Devs::adcController, 2, typename Devs::sigrow, bus_type>;

    using s0pp = servo_0::PositionProvider;
    using s1pp = servo_1::PositionProvider;
    
    using sensor = External::SPort::Sensor<External::SPort::SensorId::ID1, sensorUsart, systemTimer, Meta::List<tempiP, s0pp, s1pp, VersionProvider>>;
    
    inline static void init(const bool inverted) {
        devs::eeprom::init();
        if (!((devs::eeprom::data().magic() == 42))) {
            devs::eeprom::data().clear();
            devs::eeprom::data().change();
        }            
        
        devs::lut2::init(std::byte{0x0f}); // route TXD (inverted) to lut1-out 
        sensor::init(); 
        sensor::uart::txPinDisable(); //error on first revision of board
        
        devs::ibt::init();
        devs::ibt::off();
        
        servo::template init<AVR::BaudRate<100000>, AVR::FullDuplex, true, 1>(); // 8E2
        if (inverted) {
            servo::rxInvert(true);
            etl::outl<terminal>("SB I"_pgm);
        }
        else {
            etl::outl<terminal>("SB"_pgm);
        }
        
        devs::adcController::init();
        devs::adc::nsamples(4);
        
        devs::pwm0_2::init();
        devs::pwm3::init();
//        devs::pwm4::init(); // later initialised
//        devs::pwm5::init();
        
        servo_0::init();
        servo_1::init();
        
        esc0::init();
        esc1::init();
    }
    
    static inline void onPulse() {
        devs::pwm3::onReload([]{
            devs::evrouter::template strobe<0>();
        });
        devs::pwm4::onReload([]{
            devs::evrouter::template strobe<1>();
        });
        devs::pwm5::onReload([]{
            devs::evrouter::template strobe<2>();
        });
    }
};

template<typename Devs>
struct BusDevs<External::Bus::SumDHott<Devs>> {
    static inline uint8_t magic = 44;
    using bus_type = External::Bus::SumDHott<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
    using servo = AVR::Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
       
#ifdef LOG_OUTPUT
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif

    using configProvider = void;
    
    using sensor = Hott::Experimental::Sensor<typename Devs::sensorPosition, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg, Hott::TextMsg, systemTimer>;

    using calibratePs = Meta::List<>;

};

template<typename Devs>
struct BusDevs<External::Bus::NoBus<Devs>> {
    static inline uint8_t magic = 44;
    using bus_type = External::Bus::SumDHott<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    using systemTimer = Devs::systemTimer;

    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using servo = AVR::Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif
    
};
