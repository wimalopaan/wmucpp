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
};

template<typename Devs>
struct BusDevs<External::Bus::NoBus<Devs>> {
    using bus_type = External::Bus::NoBus<Devs>;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    
    using devs = Devs;
};
