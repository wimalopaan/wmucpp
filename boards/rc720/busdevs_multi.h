#pragma once

#include <mcu/avr.h>

#include <mcu/internals/usart.h>

#include <external/sbus/sbus.h>
#include <external/ibus/ibus2.h>

#include "fsm.h"
#include "version.h"


template<typename Bus>
struct BusDevs;

template<typename Devs>
struct BusDevs<External::Bus::IBusIBus<Devs>> {
    using bus_type = External::Bus::IBusIBus<Devs>;
    using devs = Devs;
    
    using evrouter = devs::evrouter;
    
    struct BusParam {
        using bus_t = bus_type;
        using proto_type = RCSwitch::Protocol2<RCSwitch::High>;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
    using servo = AVR::Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif

    using eeprom = EEProm::Controller<Storage::ApplDataBus<typename servo_pa::channel_t, RCSwitch::addr_t, bus_type>>;
    
    using adcController = devs::adcController;

    using value_t = servo_pa::value_type;
    using pwm0 = PPMAdapter<typename devs::pwm0_2, 1>;
    using pwm2 = PPMAdapter<typename devs::pwm0_2, 0>;    
    using pwm1 = PPMAdapter<typename devs::pwm0_2, 2>;
    using pwm3 = PPMAsyncAdapter<typename devs::pwm3>;
    using pwm4 = PPMAsyncAdapter<typename devs::pwm4>;
    using pwm5 = PPMAsyncAdapter<typename devs::pwm5>;
    
    using fsm1 = FSM<pwm0, servo_pa, eeprom, 0>;
    using fsm2 = FSM<pwm1, servo_pa, eeprom, 1>;
    using fsm3 = FSM<pwm2, servo_pa, eeprom, 2>;
    using fsm4 = FSM<pwm3, servo_pa, eeprom, 3>;
    using fsm5 = FSM<pwm4, servo_pa, eeprom, 4>;
    
    using fsms = Meta::List<fsm1, fsm2, fsm3>;
    using fsms2 = Meta::List<fsm1, fsm2, fsm3, fsm4, fsm5>;
    
    using bus_switch = RCSwitch::MultiAdapter<BusParam, servo_pa, fsms2, eeprom>;
    
    
    using tempiP = External::InternalTempProvider<typename Devs::adcController, 2, typename Devs::sigrow, bus_type>;

    using sensor = IBus2::Sensor<typename Devs::sensorPosition, AVR::Usart, AVR::BaudRate<115200>, 
                                Meta::List<tempiP, VersionProvider>, 
                                systemTimer, typename devs::ibt>;

    struct Reloader {
        static inline void init() {
            fsm4::init();
            fsm5::init();
        }
        static inline void reload() {
            devs::pwm3::onReload([]{
                fsm4::update();
                evrouter::template strobe<0>();
            });
            devs::pwm4::onReload([]{
                fsm5::update();
                evrouter::template strobe<1>();
            });
        }
    };
    using reloader = Reloader;
    
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
        
    }

//    static inline void onPulse() {
//        devs::pwm3::onReload([]{
//            devs::evrouter::template strobe<0>();
//        });
//        devs::pwm4::onReload([]{
//            devs::evrouter::template strobe<1>();
//        });
//        devs::pwm5::onReload([]{
//            devs::evrouter::template strobe<2>();
//        });
//    }
    
};

template<typename Devs>
struct BusDevs<External::Bus::SBusSPort<Devs>> {
    using bus_type = External::Bus::SBusSPort<Devs>;
    using devs = Devs;
    
    using evrouter = devs::evrouter;
    
    struct BusParam {
        using bus_t = bus_type;
        using proto_type = RCSwitch::Protocol2<RCSwitch::Low>;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using servo = AVR::Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    
#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif
    
    template<typename PA>
    using sensorUsart = AVR::Usart<typename Devs::sensorPosition, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<64>>;

    using eeprom = EEProm::Controller<Storage::ApplDataBus<typename servo_pa::channel_t, RCSwitch::addr_t, bus_type>>;
    
    using adcController = devs::adcController;
    
    using value_t = servo_pa::value_type;
    using pwm0 = PPMAdapter<typename devs::pwm0_2, 1>;
    using pwm2 = PPMAdapter<typename devs::pwm0_2, 0>;    
    using pwm1 = PPMAdapter<typename devs::pwm0_2, 2>;
    using pwm3 = PPMAsyncAdapter<typename devs::pwm3>;    
    using pwm4 = PPMAsyncAdapter<typename devs::pwm4>;
    using pwm5 = PPMAsyncAdapter<typename devs::pwm5>;

    using fsm1 = FSM<pwm0, servo_pa, eeprom, 0>;
    using fsm2 = FSM<pwm1, servo_pa, eeprom, 1>;
    using fsm3 = FSM<pwm2, servo_pa, eeprom, 2>;
    using fsm4 = FSM<pwm3, servo_pa, eeprom, 3>;
    using fsm5 = FSM<pwm4, servo_pa, eeprom, 4>;
    using fsm6 = FSM<pwm5, servo_pa, eeprom, 5>;
    
    using fsms = Meta::List<fsm1, fsm2, fsm3>;
    using fsms2 = Meta::List<fsm1, fsm2, fsm3, fsm4, fsm5, fsm6>;
    
    using bus_switch = RCSwitch::MultiAdapter<BusParam, servo_pa, fsms2, eeprom>;
    
    using tempiP = External::InternalTempProvider<typename Devs::adcController, 2, typename Devs::sigrow, bus_type>;

    using sensor = External::SPort::Sensor<External::SPort::SensorId::ID1, sensorUsart, systemTimer, Meta::List<tempiP, VersionProvider>>;

    struct Reloader {
        static inline void init() {
            fsm4::init();
            fsm5::init();
            fsm6::init();
        }
        static inline void reload() {
            devs::pwm3::onReload([]{
                fsm4::update();
                evrouter::template strobe<0>();
            });
            devs::pwm4::onReload([]{
                fsm5::update();
                evrouter::template strobe<1>();
            });
            devs::pwm5::onReload([]{
                fsm6::update();
                evrouter::template strobe<2>();
            });
        }
    };
    using reloader = Reloader;
    
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
            etl::outl<terminal>("xxSB I"_pgm);
        }
        else {
            etl::outl<terminal>("xxSB"_pgm);
        }
        
        devs::adcController::init();
        devs::adc::nsamples(4);
        
        devs::pwm0_2::init();
//        devs::pwm3::init();
//        devs::pwm4::init(); // later initialised
//        devs::pwm5::init();
        
    }
    
//    static inline void onPulse() {
//        devs::pwm3::onReload([]{
//            devs::evrouter::template strobe<0>();
//        });
//        devs::pwm4::onReload([]{
//            devs::evrouter::template strobe<1>();
//        });
//        devs::pwm5::onReload([]{
//            devs::evrouter::template strobe<2>();
//        });
//    }
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
       
#ifndef NDEBUG
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
