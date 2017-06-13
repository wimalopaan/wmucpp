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

// sudo avrdude -p atmega328pb -P usb -c avrisp2 -U lfuse:w:0xe0:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

#include <stdlib.h>

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/swusart.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/pinchange.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/mcuppm.h"
#include "mcu/avr/mcupwm.h"
#include "hal/event.h"
#include "hal/alarmtimer.h"
#include "hal/constantrate.h"
#include "hal/bufferedstream.h"
#include "hal/softspimaster.h"
#include "hal/softppm.h"
#include "external/onewire.h"
#include "external/ds18b20.h"
#include "external/ws2812.h"
#include "external/tle5205.h"
#include "external/rpm.h"
#include "external/hott/hott.h"
#include "container/stringbuffer.h"
#include "appl/blink.h"
#include "appl/exponential.h"

#include "console.h"

#define MEM
#ifdef MEM
#include "util/memory.h"
#endif

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using PortE = AVR::Port<DefaultMcuType::PortRegister, AVR::E>;

using ledPin = AVR::Pin<PortD, 4>;
using led = WS2812<1, ledPin, ColorSequenceGRB>;
typedef led::color_type Color;

using leds1Pin = AVR::Pin<PortE, 0>;
using leds1 = WS2812<8, leds1Pin, ColorSequenceGRB, false>;
typedef leds1::color_type Color1;

using leds2Pin = AVR::Pin<PortB, 0>;
using leds2 = WS2812<8, leds2Pin, ColorSequenceGRB, false>;
typedef leds2::color_type Color2;

using Tle5205Error = AVR::Pin<PortD, 2>;
using hbridge = TLE5205Hard<0, Tle5205Error>; // timer 0

using reflexPin = AVR::Pin<PortC, 3>;
using reflexPinSet = AVR::PinSet<reflexPin>;
using reflexPinChange = AVR::PinChange<reflexPinSet>;

using rpmTimer = AVR::Timer16Bit<3>; // timer 3
constexpr std::RPM MaximumRpm{12000};
using rpm = RpmFromInterruptSource<reflexPinChange, rpmTimer, MaximumRpm>;

using SoftSPIData = AVR::Pin<PortC, 4>;
using SoftSPIClock = AVR::Pin<PortC, 5>;
using SoftSPISS = AVR::Pin<PortD, 3>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;
using terminalDevice = SSpi0;
using terminal = std::basic_ostream<terminalDevice>;

using systemClock = AVR::Timer8Bit<2>; // timer 2
using alarmTimer = AlarmTimer<systemClock>;

using systemConstantRate = ConstantRateAdapter<1, void, AVR::ISR::Timer<2>::CompareA, alarmTimer>;

using hardPpm = AVR::PPM<1>; // timer1

using oneWirePin = AVR::Pin<PortD, 7>;
using oneWireMaster = OneWire::Master<oneWirePin, OneWire::Normal>;
using oneWireMasterAsync = OneWire::MasterAsync<oneWireMaster, Hott::hottDelayBetweenBytes>;
using ds18b20 = DS18B20<oneWireMasterAsync>;

std::array<OneWire::ow_rom_t, 5> dsIds;
std::array<OneWire::ow_rom_t, 5> tempIds;

using sensorRateTimer = AVR::Timer16Bit<4>; // timer 4

using sensorUsart = AVR::Usart<0, Hott::SensorProtocollAdapter<0>> ;
using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>>;

using sensorData = Hott::SensorProtocollBuffer<0>;
using menuData = Hott::SensorTextProtocollBuffer<0>;
using crWriterSensorBinary = ConstanteRateWriter<sensorData, sensorUsart>;
using crWriterSensorText = ConstanteRateWriter<menuData, sensorUsart>;
using crAdapterHott = ConstantRateAdapter<2, sensorRateTimer, AVR::ISR::Timer<4>::CompareA, 
                                          crWriterSensorBinary, crWriterSensorText, oneWireMasterAsync>;


using isrRegistrar = IsrRegistrar<systemConstantRate, 
                                  rpm,
                                  crAdapterHott, 
                                  sensorUsart::RxHandler, sensorUsart::TxHandler, rcUsart::RxHandler, rcUsart::TxHandler
>;


namespace Constant {
static constexpr std::hertz pwmFrequency = 100_Hz;
static constexpr Color cOff{0};
static constexpr Color cRed{Red{32}};
static constexpr Color cBlue{Blue{32}};
static constexpr Color cGreen{Green{32}};

static constexpr auto title = "RC SensorLed 0.1"_pgm;


}

using statusLed = Blinker<led, Constant::cGreen>;

namespace std {
std::basic_ostream<terminalDevice> cout;
std::lineTerminator<CRLF> endl;
}

const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);
const auto measurementTimer = alarmTimer::create(100_ms, AlarmFlags::Periodic);
const auto tTimer = alarmTimer::create(5000_ms, AlarmFlags::Periodic);
const auto mTimer = alarmTimer::create(750_ms, AlarmFlags::OneShot | AlarmFlags::Disabled);

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(std::byte b) {
        auto timer = std::to_integer<uint7_t>(b);
        if (timer == *periodicTimer) {
            static uint8_t counter = 0;
            ++counter;
            statusLed::tick();
           
            menuData::text()[1].insertAt(0, "bla");

            StringBufferPart<15, 4> sbv(menuData::text()[1]);
            BufferDevice sbvDev(sbv);
            sbvDev << counter;
            
            uint8_t line = 2;
            for(const auto& id: tempIds) {
                menuData::text()[line].insertAt(0, "Temp: "_pgm);
                StringBufferPart<10, 4> sbv(menuData::text()[line]);
                BufferDevice sbvDev(sbv);
                sbvDev << id.familiy();
                ++line;
            }            
            
#ifdef MEM
            std::cout << "unused: "_pgm << Util::Memory::getUnusedMemory() << std::endl;   
#endif
        }
        if (timer == *measurementTimer) {
            static ExponentialFilter<32, uint16_t> filter;
            auto r = rpm::rpm();
            rpm::reset();
            
            if (r) {
                std::RPM r2{filter(r.value())};
                Hott::SensorProtocollBuffer<0>::rpm1(r2);
            }
            else {
                static uint8_t counter = 0;
                if (++counter > 10) {
                    counter = 0;
                    filter.clear();
                    Hott::SensorProtocollBuffer<0>::rpm1(std::RPM{0});
                }
            }
        }
        else if (timer == *tTimer) {
            if (ds18b20::convert()) {
                alarmTimer::start(*mTimer);
            }
        }
        else if (timer == *mTimer) {
            ds18b20::startGet(dsIds[0]);
        }
        return true;
    }
};
struct HottBinaryHandler : public EventHandler<EventType::HottBinaryRequest> {
    static bool process(std::byte t) {
        crWriterSensorBinary::enable<true>();
        crWriterSensorText::enable<false>();
        crAdapterHott::start();
        return true;
    }
};

struct HottKeyHandler : public EventHandler<EventType::HottAsciiKey> {
    static bool process(std::byte v) {
        std::outl<terminal>("k: "_pgm, v);
        return true;
    }
};

struct HottBroadcastHandler : public EventHandler<EventType::HottSensorBroadcast> {
    static bool process(std::byte) {
//        std::cout << "hbr"_pgm << std::endl;
        crWriterSensorBinary::enable<true>();
        crWriterSensorText::enable<false>();
        crAdapterHott::start();
        return true;
    }
};

struct HottTextHandler : public EventHandler<EventType::HottAsciiRequest> {
    static bool process(std::byte) {
//        std::cout << "hba"_pgm << std::endl;
        crWriterSensorBinary::enable<false>();
        crWriterSensorText::enable<true>();
        crAdapterHott::start();
        return true;
    }
};
struct Usart0Handler : public EventHandler<EventType::UsartRecv0> {
    static bool process(std::byte) {
        statusLed::blink(Blue{32}, 2);
        return true;
    }
};
struct Usart1Handler : public EventHandler<EventType::UsartRecv1> {
    static bool process(std::byte) {
        statusLed::blink(Blue{32}, 2);
        return true;
    }
};
struct UsartFeHandler : public EventHandler<EventType::UsartFe> {
    static bool process(std::byte) {
        statusLed::blink(Color{Red{32}, Green{0}, Blue{32}}, 3);
        return true;
    }
};
struct UsartUpeHandler : public EventHandler<EventType::UsartUpe> {
    static bool process(std::byte) {
        statusLed::blink(Color{Red{32}, Green{0}, Blue{32}}, 4);
        return true;
    }
};
struct UsartDorHandler : public EventHandler<EventType::UsartDor> {
    static bool process(std::byte) {
        statusLed::blink(Color{Red{32}, Green{0}, Blue{32}}, 5);
        return true;
    }
};

struct HBridgeError : public EventHandler<EventType::TLE5205Error> {
    static bool process(std::byte) {
        statusLed::blink(Color{Red{0}, Green{32}, Blue{32}}, 5);
        return true;
    }
};

struct TWIHandlerError: public EventHandler<EventType::TWIError> {
    static bool process(std::byte) {
        std::cout << "twi error"_pgm << std::endl;
        return true;
    }  
};

struct DS18B20MeasurementHandler: public EventHandler<EventType::DS18B20Measurement> {
    static bool process(std::byte) {
        auto t = ds18b20::temperature();
        std::cout << "t: " << t << std::endl;
        Hott::SensorProtocollBuffer<0>::temp1(t);
        return true;
    }
};
struct DS18B20ErrorHandler: public EventHandler<EventType::DS18B20Error> {
    static bool process(std::byte) {
        std::cout << "t: error" << std::endl;
        return true;
    }
};

using namespace std::literals::quantity;

void updateControls() {
    static uint8_t counter = 0;
    std::percent pv1 = std::scale(Hott::SumDProtocollAdapter<0>::value8Bit(0),
                           Hott::SumDMsg::Low8Bit, Hott::SumDMsg::High8Bit);
    hbridge::pwm(pv1);
    std::percent pv2 = std::scale(Hott::SumDProtocollAdapter<0>::value8Bit(2),
                           Hott::SumDMsg::Low8Bit, Hott::SumDMsg::High8Bit);
    hardPpm::ppm<hardPpm::A>(pv2);
    std::percent pv3 = std::scale(Hott::SumDProtocollAdapter<0>::value8Bit(3),
                           Hott::SumDMsg::Low8Bit, Hott::SumDMsg::High8Bit);
    hardPpm::ppm<hardPpm::B>(pv3);
    
    Color1 c1{Red{std::expand(pv2, (uint8_t)0, (uint8_t)128)}};
    if (++counter == 0) {
        leds1::set(c1);
    }
    else if (counter == 1) {
        leds2::set(c1);
    }
}

int main() {
    isrRegistrar::init();
    alarmTimer::init();
    hbridge::init<Constant::pwmFrequency>();
    terminalDevice::init<0>();
    rpm::init();
    
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    constexpr auto tsd = AVR::Util::calculate<sensorRateTimer>(fCr);
    static_assert(tsd, "wrong parameter");
    sensorRateTimer::prescale<tsd.prescaler>();
    sensorRateTimer::ocra<tsd.ocr>();
    
    crAdapterHott::init();

    sensorUsart::init<19200>();
    rcUsart::init<115200>();
    
    leds1::init();
    leds1::set(Constant::cBlue);
    
    leds2::init();
    leds2::set(Constant::cBlue);
    
    led::init();
    led::off();    
    
    hardPpm::init();
    ds18b20::init();

    menuData::text()[0].insertAt(0, Constant::title);
    
    {
        Scoped<EnableInterrupt<>> ei;

        hbridge::pwm(0_ppc);
        hbridge::direction() = hbridge::Direction{false};
        
        std::cout << Constant::title << std::endl;
    
        oneWireMaster::findDevices(dsIds);
        uint8_t tsNumber = 0;
        for(const auto& id : dsIds) {
            std::cout << id << std::endl;
            if (id.familiy() == ds18b20::family) {
                tempIds[tsNumber++] = id;
            }
        }

        using allEventHandler = EventHandlerGroup<
                                  TimerHandler,
                                  HBridgeError,
                                  UsartFeHandler, UsartUpeHandler, UsartDorHandler, Usart0Handler, Usart1Handler,
                                  HottBinaryHandler, HottBroadcastHandler, HottTextHandler, HottKeyHandler
#ifdef I2C
                                , TWIHandlerError 
                                , ds1307, DS1307handler, DS1307handlerError
#endif
                                , ds18b20
                                , DS18B20MeasurementHandler, DS18B20ErrorHandler
        >;
        
        EventManager::run2<allEventHandler>([](){
            updateControls();
            systemConstantRate::periodic();
            crAdapterHott::periodic();
            if (EventManager::unprocessedEvent()) {
                statusLed::blink(Constant::cRed, 10);
            }
            if (EventManager::leakedEvent()) {
                statusLed::blink(Constant::cBlue, 10);
            }
        });
    }
}

// Timer 0
// PWM
//ISR(TIMER0_COMPA_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
//}

// Timer 1
// PPM
//ISR(TIMER1_COMPA_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
//}

// Timer 2
// SystemClock
ISR(TIMER2_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<2>::CompareA>();
}
//ISR(TIMER2_OVF_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<2>::Overflow>();
//}

// Timer 3
// RPM
//ISR(TIMER3_COMPA_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<3>::CompareA>();
//}
//ISR(TIMER3_COMPB_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<3>::CompareB>();
//}
//ISR(TIMER3_CAPT_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<3>::Capture>();
//}

// Timer 4
// Sensor Constant Rate
ISR(TIMER4_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<4>::CompareA>();
}

// RPM
ISR(PCINT1_vect) {
    isrRegistrar::isr<AVR::ISR::PcInt<1>>();
}

// Sensor
ISR(USART0_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART0_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}

// SumD
ISR(USART1_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
}
ISR(USART1_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif
