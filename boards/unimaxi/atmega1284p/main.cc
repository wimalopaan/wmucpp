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
#define OneWire
#define DCF
//#define I2C
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

#ifdef OneWire
# include "external/onewire.h"
# include "external/ds18b20.h"
#endif

#ifdef I2C
# include "external/ds1307.h"
#endif

#ifdef HOTT
# include "external/hott/hott.h"
#endif

#include "console.h"

#ifdef MEM
# include "util/memory.h"
#endif

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

#ifdef OneWire
using oneWirePin = AVR::Pin<PortC, 7>;
using oneWireMaster = OneWire::Master<oneWirePin, OneWire::Normal>;
using oneWireMasterAsync = OneWire::MasterAsync<oneWireMaster, constantRatePeriod>;
using ds18b20 = DS18B20<oneWireMasterAsync>;
std::array<OneWire::ow_rom_t, 5> dsIds;
#endif

using devicesConstantRateAdapter = ConstantRateAdapter<constantRateTimer, AVR::ISR::Timer<1>::CompareA 
#ifdef OneWire
                                                        ,oneWireMasterAsync
#endif
>;

using bufferedTerminal = BufferedStream<SSpi0, 512>;
using systemConstantRateAdapter = ConstantRateAdapter<void, AVR::ISR::Timer<0>::CompareA
                                                    , alarmTimer
                                                    , dcfDecoder>;

namespace std {
    std::basic_ostream<bufferedTerminal> cout;
    std::lineTerminator<CRLF> endl;
}

using isrRegistrar = IsrRegistrar<systemConstantRateAdapter
, devicesConstantRateAdapter
>;

const auto periodicTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);

struct TimerHandler : public EventHandler<EventType::Timer> {
    static void process(uint8_t timer) {
        static uint8_t counter = 0;
        ppm4out::toggle();
        if (timer == *periodicTimer) {
            ++counter;
            if (counter % 2) {
                statusLed::off();
            }
            else {
                cRGB c = {128, 0, 0};
                statusLed::set(c);
            }
#ifdef MEM
            std::cout << "unused memory: "_pgm << Util::Memory::getUnusedMemory() << std::endl;   
#endif
        }
    }
};

template<typename... Components>
struct Initializer {
    static void init() {
        (Components::init(), ...);
    }
};

struct DCFReceive0Handler : public EventHandler<EventType::DCFReceive0> {
    static void process(uint8_t n) {
        std::cout << "dcf 0 : "_pgm << n << std::endl;
    }  
};
struct DCFReceive1Handler : public EventHandler<EventType::DCFReceive1> {
    static void process(uint8_t n) {
        std::cout << "dcf 1 : "_pgm << n << std::endl;
    }  
};
struct DCFSyncHandler : public EventHandler<EventType::DCFSync> {
    static void process(uint8_t) {
        std::cout << "dcf sync  "_pgm << dcfDecoder::dateTime() << std::endl;
    }  
};
struct DCFErrorHandler : public EventHandler<EventType::DCFError> {
    static void process(uint8_t) {
        std::cout << "dcf error"_pgm << std::endl;
    }  
};
struct DCFParityHandler : public EventHandler<EventType::DCFParityError> {
    static void process(uint8_t) {
        std::cout << "dcf parity error"_pgm << std::endl;
    }  
};

int main() {
    Initializer<isrRegistrar, statusLed, leds, systemConstantRateAdapter, 
            bufferedTerminal>::init();
    statusLed::off();
    leds::off();
    SSpi0::init();
    dcfDecoder::init();
    
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
    {
        Scoped<EnableInterrupt> interruptEnabler;
        
        using allEventHandler = EventHandlerGroup<TimerHandler
#ifdef DCF
                                , DCFReceive0Handler, DCFReceive1Handler, DCFSyncHandler, DCFErrorHandler, DCFParityHandler
#endif

        >;
        
        std::cout << "UniMaxi 0.1"_pgm << std::endl;

        EventManager::run2<allEventHandler>([](){
            bufferedTerminal::periodic();
            systemConstantRateAdapter::periodic();
//            devicesConstantRateAdapter::periodic();
        });
    }
}

ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}
ISR(TIMER1_COMPA_vect) {
    ppm3out::toggle();
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif
