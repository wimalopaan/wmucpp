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

// 20MHz full-swing
// sudo avrdude -p atmega1284P -P usb -c avrisp2 -U lfuse:w:0xf7:m -U hfuse:w:0xd1:m -U efuse:w:0xfc:m

#define MEM
#define OW
#define DCF
#define I2C
//#define HOTT

#include "mcu/ports.h"
#include "mcu/avr/isr.h"

#include "hal/softspimaster.h"
#include "hal/bufferedstream.h"
#include "hal/alarmtimer.h"
#include "hal/constantrate.h"

#include "external/ws2812.h"

#ifdef DCF
# include "external/dcf77.h"
#endif

#ifdef OW
# include "external/onewire.h"
# include "external/ds18b20.h"
#endif

#ifdef I2C
# include "external/ds1307.h"
# include "external/i2cram.h"
#endif

#ifdef HOTT
# include "external/hott/hott.h"
#endif

#include "console.h"

#ifdef MEM
# include "util/memory.h"
#endif

template<typename... Components>
struct Initializer {
    static void init() {
        (Components::init(), ...);
    }
};

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using SoftSPIData = AVR::Pin<PortB, 1>;
using SoftSPIClock = AVR::Pin<PortB, 0>;
using SoftSPISS = AVR::Pin<PortC, 3>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

// Timer0
using systemTimer = AVR::Timer8Bit<0>;
using alarmTimer = AlarmTimer<systemTimer>;

// Timer1
using constantRateTimer = AVR::Timer16Bit<1>;
#ifdef HOTT
constexpr const auto constantRatePeriod = Hott::hottDelayBetweenBytes;
#else
constexpr const auto constantRatePeriod = 1000_us;
#endif

// Timer2

// Timer3

using statusLed = WS2812<1, AVR::Pin<PortC, 6>>;
using leds = WS2812<4, AVR::Pin<PortC, 4>>;
using ppm4out = AVR::Pin<PortA, 4>;
using ppm3out = AVR::Pin<PortA, 5>;

using select1 = AVR::Pin<PortC, 5>;

#ifdef DCF
using dcfPin = select1;
using dcfDecoder = DCF77<dcfPin, Config::Timer::frequency, EventManager, true>;
#endif

#ifdef OW
using oneWirePin = AVR::Pin<PortC, 7>;
using oneWireMaster = OneWire::Master<oneWirePin, OneWire::Normal>;
//using oneWireMaster = OneWire::Master<oneWirePin, OneWire::Normal, true, true>;
using oneWireMasterAsync = OneWire::MasterAsync<oneWireMaster, constantRatePeriod>;
using ds18b20 = DS18B20<oneWireMasterAsync>;
std::array<OneWire::ow_rom_t, 5> dsIds;
#endif

#ifdef I2C
using TwiMaster = TWI::Master<0>;
using TwiMasterAsync = TWI::MasterAsync<TwiMaster>;
using ds1307 = DS1307<TwiMasterAsync>;

struct MCP23008Parameter {
    static constexpr EventType eventValueAvailable = EventType::I2CRamValueAvailable;
    static constexpr EventType eventError = EventType::I2CRamError;
};
constexpr TWI::Address mcp23008Address{35};
using mcp23008 = I2CGeneric<TwiMasterAsync, mcp23008Address, MCP23008Parameter>;

constexpr TWI::Address lcdAddress{0x59};
using lcd = I2CGeneric<TwiMasterAsync, lcdAddress>;

#endif

using devicesConstantRateAdapter = ConstantRateAdapter<constantRateTimer, AVR::ISR::Timer<1>::CompareA 
#ifdef OW
                                                        ,oneWireMasterAsync
#endif
>;

using bufferedTerminal = BufferedStream<SSpi0, 512>;
using systemConstantRateAdapter = ConstantRateAdapter<void, AVR::ISR::Timer<0>::CompareA
                                                    , alarmTimer
#ifdef DCF
                                                    , dcfDecoder
#endif
>;

struct I2CInterrupt : public IsrBaseHandler<AVR::ISR::Int<2>> {
    static void isr() {
        EventManager::enqueue({EventType::ExternalInterrupt, 2});
    }
};


namespace std {
    std::basic_ostream<bufferedTerminal> cout;
    std::lineTerminator<CRLF> endl;
}

using isrRegistrar = IsrRegistrar<
                                  systemConstantRateAdapter
                                , devicesConstantRateAdapter
                                , I2CInterrupt
>;

const auto periodicTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);
#ifdef OW
const auto temperaturTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);
const auto measurementTimer = alarmTimer::create(750_ms, AlarmFlags::OneShot | AlarmFlags::Disabled);
#endif


struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(uint8_t timer) {
        static uint8_t counter = 0;
        ppm4out::toggle();
        if (timer == *periodicTimer) {
            ++counter;
            if (counter % 2) {
                statusLed::off();
#ifdef I2C
                ds1307::startReadTimeInfo();
                mcp23008::startWrite(0x09, ~counter);
                
                lcd::startWrite(0, 'a' + (counter % 10));
#endif
#ifdef OW
                
#endif
            }
            else {
                cRGB c = {128, 0, 0};
                statusLed::set(c);
            }
#ifdef MEM
            std::cout << "unused memory: "_pgm << Util::Memory::getUnusedMemory() << std::endl;   
#endif
        }
#ifdef OW
        if (timer == *temperaturTimer) {
            if (!alarmTimer::isActive(*measurementTimer)) {
                if (ds18b20::convert()) {
                    std::cout << "start ds18b20"_pgm << std::endl;
                    alarmTimer::start(*measurementTimer);
                }
            }
        }
        if (timer == *measurementTimer) {
            std::cout << "get ds18b20"_pgm << std::endl;
            ds18b20::startGet(dsIds[0]);
        }
#endif
        return true;
    }
};

#ifdef I2C
struct DS1307handler: public EventHandler<EventType::DS1307TimeAvailable> {
    static bool process(uint8_t) {
        std::cout << "ds1307 time"_pgm << std::endl;
        return true;
    }  
};

struct DS1307handlerError: public EventHandler<EventType::DS1307Error> {
    static bool process(uint8_t) {
        std::cout << "ds1307 error"_pgm << std::endl;
        return true;
    }  
};

struct TWIHandlerError: public EventHandler<EventType::TWIError> {
    static bool process(uint8_t) {
        std::cout << "twi error"_pgm << std::endl;
        return true;
    }  
};
#endif

#ifdef DCF
struct DCFReceive0Handler : public EventHandler<EventType::DCFReceive0> {
    static bool process(uint8_t n) {
        std::cout << "dcf 0 : "_pgm << n << std::endl;
        return true;
    }  
};
struct DCFReceive1Handler : public EventHandler<EventType::DCFReceive1> {
    static bool process(uint8_t n) {
        std::cout << "dcf 1 : "_pgm << n << std::endl;
        return true;
    }  
};
struct DCFSyncHandler : public EventHandler<EventType::DCFSync> {
    static bool process(uint8_t) {
        std::cout << "dcf sync  "_pgm << dcfDecoder::dateTime() << std::endl;
        return true;
    }  
};
struct DCFErrorHandler : public EventHandler<EventType::DCFError> {
    static bool process(uint8_t) {
        std::cout << "dcf error"_pgm << std::endl;
        return true;
    }  
};
struct DCFParityHandler : public EventHandler<EventType::DCFParityError> {
    static bool process(uint8_t) {
        std::cout << "dcf parity error"_pgm << std::endl;
        return true;
    }  
};
#endif
#ifdef OW
struct DS18B20MeasurementHandler: public EventHandler<EventType::DS18B20Measurement> {
    static bool process(uint8_t) {
        std::cout << "temp: "_pgm << ds18b20::temperature() << std::endl;
        return true;
    }
};
struct DS18B20ErrorHandler: public EventHandler<EventType::DS18B20Error> {
    static bool process(uint8_t) {
        std::cout << "t: error"_pgm << std::endl;
        return true;
    }
};
#endif

struct ExternalInterruptHandler: public EventHandler<EventType::ExternalInterrupt> {
    static bool process(uint8_t i) {
        std::cout << "external interrupt: "_pgm << i << std::endl;
        return true;
    }
};

int main() {
    Initializer<isrRegistrar, statusLed, leds, systemConstantRateAdapter, 
            bufferedTerminal>::init();
    statusLed::off();
    leds::off();
    SSpi0::init();

#ifdef OW
    ds18b20::init();
#endif

#ifdef DCF
    dcfDecoder::init();
#endif
    
    ppm4out::dir<AVR::Output>();
    ppm3out::dir<AVR::Output>();
    
    // todo: zusammenfassen    
    constexpr std::hertz constantRateFrequency = 1 / constantRatePeriod;
    constexpr auto tsd = AVR::Util::calculate<constantRateTimer>(constantRateFrequency);
    static_assert(tsd, "wrong parameter");
    constantRateTimer::prescale<tsd.prescaler>();
    constantRateTimer::ocra<tsd.ocr>();
    
    devicesConstantRateAdapter::init();
    
    // todo: WatchDog
    
    constexpr auto interrupts = AVR::getBaseAddr<AVR::ATMega1284P::Interrupt>;
    interrupts()->eicra.set<AVR::ATMega1284P::Interrupt::EIControl::isc21>();
    interrupts()->eimsk.set<AVR::ATMega1284P::Interrupt::EIMask::int2>();
    
    {
        Scoped<EnableInterrupt> interruptEnabler;
        
        using allEventHandler = EventHandlerGroup<
                                  TimerHandler
                                , ExternalInterruptHandler
#ifdef DCF
                                , DCFReceive0Handler, DCFReceive1Handler, DCFSyncHandler, DCFErrorHandler, DCFParityHandler
#endif
#ifdef I2C
                                , TWIHandlerError 
                                , ds1307, DS1307handler, DS1307handlerError
#endif
#ifdef OW   
                                , ds18b20
                                , DS18B20MeasurementHandler, DS18B20ErrorHandler
#endif
        >;
        
        std::cout << "UniMaxi 0.1"_pgm << std::endl;

#ifdef OW
        oneWireMaster::findDevices(dsIds);
        for(const auto& id : dsIds) {
            std::cout << id << std::endl;
        }
#endif
#ifdef I2C
        TwiMaster::init<ds1307::fSCL>();
        ds1307::init();
        ds1307::halt<false>();
        ds1307::squareWave<true>();
        
        std::array<TWI::Address, 10> i2cAddresses;
        TwiMaster::findDevices(i2cAddresses);
        for(const auto& d : i2cAddresses) {
            std::cout << d << std::endl;
        }
        
        mcp23008::startWrite(0x00, 0x00); // output
#endif    
        EventManager::run2<allEventHandler>([](){
            bufferedTerminal::periodic();
            systemConstantRateAdapter::periodic();
            devicesConstantRateAdapter::periodic();
#ifdef I2C
            TwiMasterAsync::periodic();
#endif
            if (EventManager::unprocessedEvent()) {
                cRGB c = {255, 255, 255};
                statusLed::set(c);
            }
            if (EventManager::leakedEvent()) {
                cRGB c = {255, 0, 255};
                statusLed::set(c);
            }
        });
    }
}

ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}
ISR(TIMER1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}
ISR(INT2_vect) {
    isrRegistrar::isr<AVR::ISR::Int<2>>();
}


#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif
