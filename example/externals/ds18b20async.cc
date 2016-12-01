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
#include "external/ds18b20.h"
#include "hal/softspimaster.h"
#include "hal/softtimer.h"
#include "hal/event.h"
#include "container/pgmstring.h"
#include "console.h"

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using systemClock = AVR::Timer8Bit<0>;
using systemTimer = Timer<systemClock>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

using terminal = SSpi0;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

constexpr std::microseconds systimerreso = std::duration_cast<std::microseconds>(Config::Timer::resolution);

using oneWirePin = AVR::Pin<PortA, 5>;
using oneWireMaster = OneWire::Master<oneWirePin, OneWire::Normal>;
using oneWireMasterAsync = OneWire::MasterAsync<oneWireMaster, systimerreso>;
using ds18b20 = DS18B20<oneWireMasterAsync>;

std::array<OneWire::ow_rom_t, 5> dsIds;

std::optional<uint7_t> pTimer;
std::optional<uint7_t> mTimer;

struct TimerHandler : public EventHandler<EventType::Timer> {
    static void process(uint8_t timer) {
        if (timer == *pTimer) {
            std::cout << "Periodic timer"_pgm << std::endl;
            ds18b20::convert();
            mTimer = systemTimer::create(750_ms, TimerFlags::OneShot);
        }
        if (timer == *mTimer) {
            std::cout << "Measurement timer"_pgm << std::endl;
            ds18b20::startGet(dsIds[0]);
            mTimer = std::optional<uint7_t>();
        }
    }
};
struct MeasuremntHandler : public EventHandler<EventType::DS18B20Measurement> {
    static void process(uint8_t) {
        std::cout << "Temperature: "_pgm << ds18b20::temperature() << std::endl;
    }
};

struct ErrorHandler : public EventHandler<EventType::DS18B20Error> {
    static void process(uint8_t) {
        std::cout << "Error"_pgm << std::endl;        
    }
};

using periodicGroup = PeriodicGroup<systemTimer>;
using eventHandlerGroup = EventHandlerGroup<TimerHandler, MeasuremntHandler, ErrorHandler>;

int main()
{
    systemTimer::init();
    
    ds18b20::init();
    
    oneWireMaster::findDevices(dsIds);
    for(const auto& id : dsIds) {
        std::cout << id << std::endl;
    }
    
    pTimer = systemTimer::create(3000_ms, TimerFlags::Periodic);
    
    {
        Scoped<EnableInterrupt> ie;
        EventManager::run<periodicGroup, eventHandlerGroup>([](){});
    }

}
ISR(TIMER0_COMPA_vect) {
    periodicGroup::tick();
}

void assertFunction(bool b, const char* function, const char* file, unsigned int line) {
    if (!b) {
        std::cout << "Assertion failed: "_pgm << function << ","_pgm << file << ","_pgm << line << std::endl;
        abort();
    }
}
