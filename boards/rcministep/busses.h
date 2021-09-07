#pragma once

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/port.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/event.h>

#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>

#include <external/solutions/rc/busscan.h>

template<typename Bus>
struct BusDevs;

template<typename Devs>
struct BusDevs<External::Bus::IBusIBus<Devs>> {
    using bus_type = External::Bus::IBusIBus<Devs>;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    
    using devs = Devs;

    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
//    using servo = AVR::Usart<typename devs::usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<128>>;
//    using term_dev = servo;
#ifndef NDEBUG
    using servo = AVR::Usart<typename devs::usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<128>>;
    using term_dev = servo;
#else
    using servo = AVR::Usart<typename devs::usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;
    using term_dev = void;
#endif
};

template<typename Devs>
struct BusDevs<External::Bus::SBusSPort<Devs>> {
    using bus_type = External::Bus::SBusSPort<Devs>;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    
    using devs = Devs;

    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, typename devs::systemTimer>;
//    using servo = AVR::Usart<typename devs::usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<128>>;
//    using term_dev = servo;
#ifndef NDEBUG
    using servo = AVR::Usart<typename devs::usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<128>>;
    using term_dev = servo;
#else
    using servo = AVR::Usart<typename devs::usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;
    using term_dev = void;
#endif
};

template<typename Devs>
struct BusDevs<External::Bus::SumDHott<Devs>> {
    using bus_type = External::Bus::SBusSPort<Devs>;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    
    using devs = Devs;

    using servo_pa = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
//    using servo = AVR::Usart<typename devs::usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<128>>;
//    using term_dev = servo;
#ifndef NDEBUG
    using servo = AVR::Usart<typename devs::usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<128>>;
    using term_dev = servo;
#else
    using servo = AVR::Usart<typename devs::usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;
    using term_dev = void;
#endif
};

template<typename Devs>
struct BusDevs<External::Bus::NoBus<Devs>> {
    using bus_type = External::Bus::NoBus<Devs>;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    
    using devs = Devs;
};
