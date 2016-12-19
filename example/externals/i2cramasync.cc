/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>

#include "mcu/ports.h"
#include "mcu/avr/twi.h"
#include "external/i2cram.h"
#include "hal/softspimaster.h"
#include "hal/event.h"
#include "hal/softtimer.h"
#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using systemClock = AVR::Timer8Bit<0>;
using systemTimer = Timer<systemClock>;

static constexpr std::hertz fI2C{100000};
static constexpr TWI::Address i2cramAddress{0x54};
using TwiMaster = TWI::Master<0>;
using TwiMasterAsync = TWI::MasterAsync<TwiMaster>;
using i2cram = I2CRam<TwiMasterAsync, i2cramAddress>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

using terminal = SSpi0;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

struct DS1307handler: public EventHandler<EventType::I2CRamValueAvailable> {
    static void process(uint8_t v) {
        std::cout << "ram: "_pgm << v << std::endl;
    }  
};

struct DS1307handlerError: public EventHandler<EventType::I2CRamError> {
    static void process(uint8_t) {
        std::cout << "ram error"_pgm << std::endl;
    }  
};

struct TWIHandlerError: public EventHandler<EventType::TWIError> {
    static void process(uint8_t v) {
        std::cout << "twi error: "_pgm << v << std::endl;
    }  
};

struct TimerHandler : public EventHandler<EventType::Timer> {
    static void process(uint8_t) {
        std::cout << "timer"_pgm << std::endl;
        if (!i2cram::startWrite(0, 1)) {
            std::cout << "send error"_pgm << std::endl;
        }
        if (!i2cram::startRead(0)) {
            std::cout << "send error"_pgm << std::endl;
        }
    }
};

using periodicGroup = PeriodicGroup<systemTimer>;
using eventHandlerGroup = EventHandlerGroup<TimerHandler, TWIHandlerError, i2cram, DS1307handler, DS1307handlerError>;

int main()
{
    terminal::init();
    systemTimer::init();
    
    i2cram::init<fI2C>();
    
    std::cout << "I2C Ram (attiny) async example"_pgm << std::endl;

    std::array<TWI::Address, 5> i2cAddresses;
    TwiMaster::findDevices(i2cAddresses);
    for(const auto& d : i2cAddresses) {
        std::cout << d << std::endl;
    }
    
    systemTimer::create(1000_ms, TimerFlags::Periodic);
    
    {
        Scoped<EnableInterrupt> ei;
        EventManager::run<periodicGroup, eventHandlerGroup>([](){
            TwiMasterAsync::periodic();
        });
    }
}

#ifndef NDEBUG
void assertFunction(const char*, const char* function, const char* file, unsigned int line) {
    std::cout << "Assertion failed: "_pgm << function << ","_pgm << file << ","_pgm << line << std::endl;
    while(true) {};
}
#endif

ISR(TIMER0_COMPA_vect) {
    periodicGroup::tick();
}
