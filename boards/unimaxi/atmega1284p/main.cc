/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
//#define BTerm
#define I2CInt
#define LEDS1
//#define SOFTPWM
//#define HARDPWM
//#define SOFTPPM
//#define HARDPPM1
//#define HARDPPM2
//#define SWUSART

// todo:: HC-05 mit Uart verbinden
// todo:: HC-05 EN-Pin trennen

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
# include "external/ds2401.h"
# include "external/ds24b33.h"
#endif

#ifdef I2C
# include "external/ds1307.h"
# include "external/i2cram.h"
#endif

#ifdef HOTT
# include "external/hott/hott.h"
#endif

#if defined(SOFTPWM) || defined(HARDPWM)
# include "external/tle5205.h"
#endif
#if defined(HARDPPM1) || defined(HARDPPM2)
# include "mcu/avr/mcuppm.h"
#endif

#ifdef SOFTPPM
# include "hal/softppm.h"
#endif

#ifdef SWUSART
# include "mcu/avr/swusart.h"
#endif

#ifdef MEM
# include "util/memory.h"
#endif

#include "console.h"

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
using terminal = SSpi0;
using bufferedTerminal = BufferedStream<SSpi0, 512>;

// Timer0
using systemTimer = AVR::Timer8Bit<0>;
using alarmTimer = AlarmTimer<systemTimer>;

// Timer1
#ifdef HARDPWM
using hbridge = TLE5205Hard<1, void>;
#elif defined(HARDPPM1)
using ppm = AVR::PPM<1>;
#elif defined(SOFTPPM)
using ppm3out = AVR::Pin<PortA, 5>;
using ppm4out = AVR::Pin<PortA, 4>;
using ppmTimerOutput = AVR::Timer16Bit<1>;
using softPpm = SoftPPM<ppmTimerOutput, ppm3out, ppm4out>;
#elif defined(SWUSART)
using swusart = SWUsart<0>;
#endif

// Timer2
// ppm test
#ifdef HARDPPM2
using ppm = AVR::PPM<2>;
#else
# ifdef SOFTPWM
// pwm test
using Tle5205In1 = AVR::Pin<PortD, 4>;
using Tle5205In2 = AVR::Pin<PortD, 5>;
//using Tle5205Error = AVR::Pin<PortD, 2>; / select1
using tleTimer = AVR::Timer8Bit<2>; 
using hbridge = TLE5205Soft<Tle5205In1, Tle5205In2, void, tleTimer>;
# endif
#endif

// Timer3
using constantRateTimer = AVR::Timer16Bit<3>;
#ifdef HOTT
constexpr const auto constantRatePeriod = Hott::hottDelayBetweenBytes;
#else
constexpr const auto constantRatePeriod = 1000_us;
#endif

using statusLed = WS2812<1, AVR::Pin<PortC, 6>>;
typedef statusLed::color_type StatusColor;
#ifdef LEDS1
using leds = WS2812<4, AVR::Pin<PortC, 4>>;
typedef leds::color_type Color;
#endif

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
using ds2401  = DS2401<oneWireMasterAsync>;
using ds24b33 = DS24B33<oneWireMasterAsync>;

std::array<OneWire::ow_rom_t, 5> dsIds;

OneWire::ow_rom_t tSensorId;
OneWire::ow_rom_t boardId;
OneWire::ow_rom_t eepromId;

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


using devicesConstantRateAdapter = ConstantRateAdapter<constantRateTimer, AVR::ISR::Timer<constantRateTimer::number>::CompareA 
#ifdef OW
                                                        ,oneWireMasterAsync
#endif
>;

using systemConstantRateAdapter = ConstantRateAdapter<void, AVR::ISR::Timer<0>::CompareA
                                                    , alarmTimer
#ifdef DCF
                                                    , dcfDecoder
#endif
>;

#ifdef I2CInt
struct I2CInterrupt : public IsrBaseHandler<AVR::ISR::Int<2>> {
    static void isr() {
        EventManager::enqueue({EventType::ExternalInterrupt, 2});
    }
};
#endif

namespace std {
#ifdef BTerm
    std::basic_ostream<bufferedTerminal> cout;
#else
std::basic_ostream<terminal> cout;
#endif
    std::lineTerminator<CRLF> endl;
}

using isrRegistrar = IsrRegistrar<
                                  systemConstantRateAdapter
                                , devicesConstantRateAdapter
#ifdef I2CInt
                                , I2CInterrupt
#endif
#ifdef SOFTPWM
                                , hbridge::PwmOnHandler, hbridge::PwmOffHandler
#endif
#ifdef SOFTPPM
                                , softPpm::OCAHandler, softPpm::OCBHandler
#endif
#ifdef SWUSART
                                , swusart::StartBitHandler, swusart::ReceiveBitHandler, swusart::TransmitBitHandler
#endif
>;

namespace Constant {
static constexpr std::hertz pwmFrequency = 100_Hz;
}

const auto periodicTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);
#ifdef OW
const auto temperaturTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);
const auto measurementTimer = alarmTimer::create(750_ms, AlarmFlags::OneShot | AlarmFlags::Disabled);
#endif


struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(uint8_t timer) {
        static uint8_t counter = 0;
        if (timer == *periodicTimer) {
            ++counter;
#ifdef LEDS1
            Color color{Blue{10}};
            leds::set<false>(Color{0});
            leds::set(counter % 4, color);
#endif
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
                StatusColor c = {Red{128}};
                statusLed::set(c);
            }
#ifdef SWUSART
            std::basic_ostream<swusart> btStream;
            btStream << "Bla" << std::endl;
#endif
#ifdef MEM
            std::cout << "unused memory: "_pgm << Util::Memory::getUnusedMemory() << std::endl;   
#endif
        }
#ifdef OW
        if (timer == *temperaturTimer) {
            if (!alarmTimer::isActive(*measurementTimer)) {
                if (ds18b20::convert()) {
//                    std::cout << "start ds18b20"_pgm << std::endl;
                    alarmTimer::start(*measurementTimer);
                }
            }
        }
        if (timer == *measurementTimer) {
//            std::cout << "get ds18b20"_pgm << std::endl;
            ds18b20::startGet(tSensorId);
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
//        std::cout << "dcf 0 : "_pgm << n << std::endl;
        return true;
    }  
};
struct DCFReceive1Handler : public EventHandler<EventType::DCFReceive1> {
    static bool process(uint8_t n) {
//        std::cout << "dcf 1 : "_pgm << n << std::endl;
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
//        std::cout << "external interrupt: "_pgm << i << std::endl;
        return true;
    }
};

int main() {
    Initializer<isrRegistrar, statusLed, 
        #ifdef LEDS1
            leds, 
        #endif
            systemConstantRateAdapter, 
        #ifdef BTerm
            bufferedTerminal
        #else
            terminal
        #endif
            >::init();
    statusLed::off();
#ifdef LEDS1
    leds::off();
#endif
#ifdef OW
    ds18b20::init();
#endif
#ifdef I2C
    TwiMaster::init<ds1307::fSCL>();
    ds1307::init();
#endif
#ifdef DCF
    dcfDecoder::init();
#endif
#if defined(SOFTPWM) || defined(HARDPWM)
    hbridge::init<Constant::pwmFrequency>();
    hbridge::direction() = TLE5205Base::Direction{false};
    hbridge::pwm(10_ppc);
#elif defined(HARDPPM1)
    ppm::init();
//    ppm::ppm<ppm::A>(0_ppc);
//    ppm::ppm<ppm::B>(50_ppc);
#elif defined(SOFTPPM)
    softPpm::init();
#elif defined (SWUSART)
    swusart::init<9600>();
#endif
    
    
    // todo: zusammenfassen    
    constexpr std::hertz constantRateFrequency = 1 / constantRatePeriod;
    constexpr auto tsd = AVR::Util::calculate<constantRateTimer>(constantRateFrequency);
    static_assert(tsd, "wrong parameter");
    constantRateTimer::prescale<tsd.prescaler>();
    constantRateTimer::ocra<tsd.ocr>();
    
    devicesConstantRateAdapter::init();
    
    // todo: WatchDog
    
#ifdef I2CInt
    constexpr auto interrupts = AVR::getBaseAddr<AVR::ATMega1284P::Interrupt>;
    interrupts()->eicra.set<AVR::ATMega1284P::Interrupt::EIControl::isc21>();
    interrupts()->eimsk.set<AVR::ATMega1284P::Interrupt::EIMask::int2>();
#endif
    {
        Scoped<EnableInterrupt> interruptEnabler;
        
        using allEventHandler = EventHandlerGroup<
                                  TimerHandler
#ifdef I2CInt
                                , ExternalInterruptHandler
#endif
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
        
        std::cout << "UniMaxi 0.2"_pgm << std::endl;

#ifdef OW
        oneWireMaster::findDevices(dsIds);
        for(const auto& id : dsIds) {
            std::cout << id << std::endl;
            if (id.familiy() == ds18b20::family) {
                tSensorId = id;
            }
            else if(id.familiy() == ds2401::family) {
                boardId = id;
            }
            else if(id.familiy() == ds24b33::family) {
                eepromId = id;
            }
        }
        std::cout << "Temp:   " << tSensorId << std::endl;
        std::cout << "Board:  " << boardId  << std::endl;
        std::cout << "EEProm: " << eepromId << std::endl;
        
#endif
#ifdef I2C
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
#ifdef BTerm
            bufferedTerminal::periodic();
#endif
            systemConstantRateAdapter::periodic();
            devicesConstantRateAdapter::periodic();
#ifdef I2C
            TwiMasterAsync::periodic();
#endif
            if (EventManager::unprocessedEvent()) {
                StatusColor c{255};
                statusLed::set(c);
            }
            if (EventManager::leakedEvent()) {
                StatusColor c = {Red{255}, Green{0}, Blue{255}};
                statusLed::set(c);
            }
        });
    }
}

ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}
#ifdef SOFTPPM
ISR(TIMER1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}
ISR(TIMER1_COMPB_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareB>();
}
#elif defined SWUSART
ISR(TIMER1_CAPT_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::Capture>();
}
ISR(TIMER1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}
ISR(TIMER1_COMPB_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareB>();
}
#endif
#ifdef SOFTPWM
ISR(TIMER2_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<2>::Overflow>();
}
ISR(TIMER2_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<2>::CompareA>();
}
#endif
ISR(TIMER3_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<3>::CompareA>();
}
#ifdef I2CInt
ISR(INT2_vect) {
    isrRegistrar::isr<AVR::ISR::Int<2>>();
}
#endif

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif
