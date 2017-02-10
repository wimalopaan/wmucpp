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
#include "hal/event.h"
#include "hal/alarmtimer.h"
#include "hal/constantrate.h"
#include "hal/bufferedstream.h"
#include "external/ws2812.h"
#include "external/tle5205.h"
#include "external/rpm.h"
#include "external/hott/hott.h"
#include "appl/blink.h"

#include "console.h"

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using PortE = AVR::Port<DefaultMcuType::PortRegister, AVR::E>;

using ledPin = AVR::Pin<PortD, 6>;
using led = WS2812<1, ledPin, ColorSequenceGRB>;
typedef led::color_type Color;

using leds1Pin = AVR::Pin<PortE, 0>;
using leds2Pin = AVR::Pin<PortB, 0>;

using Tle5205In1 = AVR::Pin<PortD, 5>;
using Tle5205In2 = AVR::Pin<PortD, 4>;
using Tle5205Error = AVR::Pin<PortD, 2>;
using tleTimer = AVR::Timer8Bit<2>; // timer 2
using hbridge = TLE5205<Tle5205In1, Tle5205In2, Tle5205Error, tleTimer>;

using reflexPin = AVR::Pin<PortC, 3>;
using reflexPinSet = AVR::PinSet<reflexPin>;
using reflexPinChange = AVR::PinChange<reflexPinSet>;

using rpmTimer = AVR::Timer16Bit<1>; // timer 1
using rpm = RpmFromInterruptSource<reflexPinChange, rpmTimer>;

using debugUart = SWUsart<1>; // timer 3 (icp3)
using terminal = BufferedStream<debugUart, 256>;

using systemClock = AVR::Timer8Bit<0>; // timer 0
using alarmTimer = AlarmTimer<systemClock>;

using systemConstantRate = ConstantRateAdapter<void, AVR::ISR::Timer<0>::CompareA, alarmTimer>;

using sensorRateTimer = AVR::Timer16Bit<4>; // timer 4

using sensorUsart = AVR::Usart<0, Hott::SensorProtocollAdapter<0>> ;
using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>>;

using crWriterSensorBinary = ConstanteRateWriter<Hott::SensorProtocollBuffer<0>, sensorUsart>;
using crWriterSensorText = ConstanteRateWriter<Hott::SensorTextProtocollBuffer<0>, sensorUsart>;
using crAdapterHott = ConstantRateAdapter<sensorRateTimer, AVR::ISR::Timer<4>::CompareA, 
                                          crWriterSensorBinary, crWriterSensorText>;

using isrRegistrar = IsrRegistrar<systemConstantRate, 
                                  hbridge::PwmOnHandler, hbridge::PwmOffHandler,
                                  debugUart::ReceiveBitHandler, debugUart::TransmitBitHandler, debugUart::StartBitHandler,
                                  rpm,
                                  crAdapterHott, 
                                  sensorUsart::RxHandler, sensorUsart::TxHandler, rcUsart::RxHandler, rcUsart::TxHandler
>;


namespace Constant {
static constexpr std::hertz pwmFrequency = 1000_Hz;
static constexpr Color cOff{0};
static constexpr Color cRed{Red{128}};
static constexpr Color cBlue{Blue{128}};
static constexpr Color cGreen{Green{128}};
}

using statusLed = Blinker<led, Constant::cGreen>;

namespace std {
std::basic_ostream<terminal> cout;
std::lineTerminator<CRLF> endl;
}

const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(uint8_t timer) {
        if (timer == *periodicTimer) {
            static uint8_t counter = 0;
            ++counter;
            statusLed::tick();
            std::cout << "counter: "_pgm << counter << std::endl;
        }
        return true;
    }
};
struct HottBinaryHandler : public EventHandler<EventType::HottBinaryRequest> {
    static bool process(uint8_t) {
//        std::cout << "hbb"_pgm << std::endl;
        crWriterSensorBinary::enable<true>();
        crWriterSensorText::enable<false>();
        crAdapterHott::start();
        return true;
    }
};

struct HottKeyHandler : public EventHandler<EventType::HottAsciiKey> {
    static bool process(uint8_t v) {
        std::cout << "k: "_pgm << v << std::endl;
//        Hott::SensorProtocoll<sensorUsart>::key(v);
        return true;
    }
};

struct HottBroadcastHandler : public EventHandler<EventType::HottSensorBroadcast> {
    static bool process(uint8_t) {
        std::cout << "hbr"_pgm << std::endl;
        crWriterSensorBinary::enable<true>();
        crWriterSensorText::enable<false>();
        crAdapterHott::start();
        return true;
    }
};

struct HottTextHandler : public EventHandler<EventType::HottAsciiRequest> {
    static bool process(uint8_t) {
        std::cout << "hba"_pgm << std::endl;
        crWriterSensorBinary::enable<false>();
        crWriterSensorText::enable<true>();
        crAdapterHott::start();
        return true;
    }
};
struct Usart0Handler : public EventHandler<EventType::UsartRecv0> {
    static bool process(uint8_t) {
        statusLed::blink(Blue{32}, 2);
        return true;
    }
};
struct Usart1Handler : public EventHandler<EventType::UsartRecv1> {
    static bool process(uint8_t) {
        statusLed::blink(Blue{32}, 2);
        return true;
    }
};
struct UsartFeHandler : public EventHandler<EventType::UsartFe> {
    static bool process(uint8_t) {
        statusLed::blink(Color{Red{32}, Green{0}, Blue{32}}, 3);
        return true;
    }
};
struct UsartUpeHandler : public EventHandler<EventType::UsartUpe> {
    static bool process(uint8_t) {
        statusLed::blink(Color{Red{32}, Green{0}, Blue{32}}, 4);
        return true;
    }
};
struct UsartDorHandler : public EventHandler<EventType::UsartDor> {
    static bool process(uint8_t) {
        statusLed::blink(Color{Red{32}, Green{0}, Blue{32}}, 5);
        return true;
    }
};


int main() {
    isrRegistrar::init();
    alarmTimer::init();
    hbridge::init<Constant::pwmFrequency>();
    terminal::init<2400>();
    rpm::init();
    
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    constexpr auto tsd = AVR::Util::calculate<sensorRateTimer>(fCr);
    static_assert(tsd, "wrong parameter");
    sensorRateTimer::prescale<tsd.prescaler>();
    sensorRateTimer::ocra<tsd.ocr>();
    
    crAdapterHott::init();

    sensorUsart::init<19200>();
    rcUsart::init<115200>();
    
    leds1Pin::template dir<AVR::Output>();
    leds2Pin::template dir<AVR::Output>();
    
    led::init();
    led::off();    
    
    {
        Scoped<EnableInterrupt> ei;

        using namespace std::literals::quantity;
        hbridge::pwm(0_ppc);
        hbridge::direction() = hbridge::Direction{false};
        
        std::cout << "RC SensorLed 0.1"_pgm << std::endl;

        using allEventHandler = EventHandlerGroup<
                                  TimerHandler,
        UsartFeHandler, UsartUpeHandler, UsartDorHandler, Usart0Handler, Usart1Handler,
        HottBinaryHandler, HottBroadcastHandler, HottTextHandler        
#ifdef I2C
                                , TWIHandlerError 
                                , ds1307, DS1307handler, DS1307handlerError
#endif
#ifdef OW   
                                , ds18b20
                                , DS18B20MeasurementHandler, DS18B20ErrorHandler
#endif
        >;
        
        EventManager::run2<allEventHandler>([](){
            terminal::periodic();
            systemConstantRate::periodic();
            crAdapterHott::periodic();
            hbridge::periodic();
            if (EventManager::unprocessedEvent()) {
                EventManager::unprocessedEvent() = false;
                statusLed::blink(Constant::cRed, 10);
            }
            if (EventManager::leakedEvent()) {
                EventManager::leakedEvent() = false;
                statusLed::blink(Constant::cBlue, 10);
            }
        });
    }
}

ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}
ISR(TIMER4_COMPA_vect) {
    leds1Pin::toggle();
    isrRegistrar::isr<AVR::ISR::Timer<4>::CompareA>();
}
ISR(TIMER2_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<2>::CompareA>();
}
ISR(TIMER2_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<2>::Overflow>();
}
ISR(TIMER3_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<3>::CompareA>();
}
ISR(TIMER3_COMPB_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<3>::CompareB>();
}
ISR(TIMER3_CAPT_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<3>::Capture>();
}
ISR(PCINT1_vect) {
    isrRegistrar::isr<AVR::ISR::PcInt<1>>();
}
ISR(USART0_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART0_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}
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
