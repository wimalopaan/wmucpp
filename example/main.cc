#include "main.h"
#include "std/limits.h"
#include "mcu/avr8.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/usart.h"
#include "hal/event.h"
#include "mcu/ports.h"
#include "units/physical.h"
#include "hal/softtimer.h"
#include "std/literals.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/spi.h"
#include "mcu/avr/swusart.h"
#include "external/ws2812.h"
#include "mcu/avr/pinchange.h"
#include "mcu/avr/ppm.h"
#include "mcu/avr/adc.h"
#include "hal/ppmswitch.h"
#include "util/fsm.h"
#include "util/delay.h"
#include "external/hott/hott.h"
#include "console.h"
#include "hal/softspimaster.h"
#include "hal/button.h"
#include "external/dcf77.h"
#include "hal/softppm.h"
#include "mcu/avr/mcupwm.h"
#include "hal/softpwm.h"
#include "hal/constantrate.h"
#include "hal/variablerate.h"
#include "hal/bufferedstream.h"
#include "hal/adccontroller.h"
#include "external/onewire.h"
#include "external/ds18b20.h"
#include "external/ds1307.h"
#include "external/lm35.h"
#include "mcu/avr/twi.h"
#include "std/array.h"

#include <stdlib.h> // abort()

// 20MHz full-swing
// sudo avrdude -p atmega1284P -P usb -c avrisp2 -U lfuse:w:0xf7:m -U hfuse:w:0xd1:m -U efuse:w:0xfc:m

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

using oneWirePin = AVR::Pin<PortA, 5>;
using oneWireMaster = OneWire::Master<oneWirePin, OneWire::Normal>;
using oneWireMasterAsync = OneWire::MasterAsync<oneWireMaster, Hott::hottDelayBetweenBytes>;
using ds18b20 = DS18B20<oneWireMasterAsync>;

std::array<OneWire::ow_rom_t, 5> dsIds;

//using terminal = SSpi0;
using bufferedTerminal = BufferedStream<SSpi0, 512>;

namespace std {
    std::basic_ostream<bufferedTerminal> cout;
//    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

using button0 = Button<0, AVR::Pin<PortA, 3>>;
using button1 = Button<1, AVR::Pin<PortA, 4>>;
using buttonController = ButtonController<button0, button1>;

using dcfPin = AVR::Pin<PortA, 7>;
using dcfDecoder = DCF77<dcfPin>;

using adc = AVR::Adc<0>;
using adcController = AdcController<adc, 6>;
using lm35 = LM35<adcController, 0>;

using TwiMaster = TWI::Master<0>;
using TwiMasterAsync = TWI::MasterAsync<TwiMaster>;
using ds1307 = DS1307<TwiMasterAsync>;

using vrAdapter = VariableRateAdapter<bufferedTerminal, adcController>;

using ws2812_A = AVR::Pin<PortC, 2>;
//using ws2812_B = AVR::Pin<PortC, 2>;

using ppmInputPin = AVR::Pin<PortC, 3>;
using pinChangeHandlerPpm = AVR::PinChange<ppmInputPin>;
using ppmTimerInput = AVR::Timer8Bit<2>;
using ppm1 = PpmDecoder<pinChangeHandlerPpm, ppmTimerInput>;
using ppmSwitch = PpmSwitch<0, ppm1>;

using systemClock = AVR::Timer8Bit<0>;
using systemTimer = Timer<systemClock>;

using sensorUsart = AVR::Usart<0, Hott::SensorProtocollAdapter<0>> ;
using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>>;


using ppmTimerOutput = AVR::Timer16Bit<3>;
using ppmPin1 = AVR::Pin<PortC, 4>;
using ppmPin2 = AVR::Pin<PortC, 5>;
using softPpm = SoftPPM<ppmTimerOutput, ppmPin1, ppmPin2>;

// todo: testen, ob das mit PpmChange vereinbar ist
using hardPwm = AVR::PWM<2>;


using crTestPin = AVR::Pin<PortB, 1>;
using crTimer = AVR::Timer16Bit<1>;
using crWriterSensorBinary = ConstanteRateWriter<Hott::SensorProtocollBuffer<0>, sensorUsart>;
using crWriterSensorText = ConstanteRateWriter<Hott::SensorTextProtocollBuffer<0>, sensorUsart>;
//using crAdapter = ConstantRateAdapter<crTimer, AVR::ISR::Timer<1>::CompareA, crWriterSensorBinary, crWriterSensorText, TestBitShifter<crTestPin, 0x55>>;
using crAdapterHott = ConstantRateAdapter<crTimer, AVR::ISR::Timer<1>::CompareA, crWriterSensorBinary, crWriterSensorText>;
using crAdapterOneWire = ConstantRateAdapter<void, AVR::ISR::Timer<1>::CompareA, oneWireMasterAsync>;

using isrDistributor = IsrDistributor<AVR::ISR::Timer<1>::CompareA, crAdapterHott, crAdapterOneWire>;

using softPwmPin1 = AVR::Pin<PortB, 2>;
using softPwmPin2 = AVR::Pin<PortB, 3>;
using softPwm = SoftPWM<softPwmPin1, softPwmPin2>;

using sampler = PeriodicGroup<buttonController, systemTimer, dcfDecoder, softPwm>; // werden alle resolution ms aufgerufen

//using testPin = AVR::Pin<PortD, 5>;

using led = AVR::Pin<PortB, 0>;

struct EventHandlerParameter {
    std::optional<uint7_t> timerId1;
};

struct DS1307handler: public EventHandler<EventType::DS1307TimeAvailable> {
    static void process(uint8_t) {
        std::cout << "ds1307 time"_pgm << std::endl;
    }  
};

struct DS1307handlerError: public EventHandler<EventType::DS1307Error> {
    static void process(uint8_t) {
        std::cout << "ds1307 error"_pgm << std::endl;
    }  
};

struct TWIHandlerError: public EventHandler<EventType::TWIError> {
    static void process(uint8_t) {
        std::cout << "twi error"_pgm << std::endl;
    }  
};

class DS18B20MeasurementHandler: public EventHandler<EventType::DS18B20Measurement> {
public:
    static void process(uint8_t) {
        std::cout << "t: " << ds18b20::temperature() << std::endl;
    }
};
class DS18B20ErrorHandler: public EventHandler<EventType::DS18B20Error> {
public:
    static void process(uint8_t) {
        std::cout << "t: error" << std::endl;
    }
};

class Button0Handler: public EventHandler<EventType::ButtonPress0> {
public:
    static void process(uint8_t) {
        std::cout << "button 0 press"_pgm << std::endl;
    }
};

class HottBinaryHandler : public EventHandler<EventType::HottBinaryRequest> {
public:
    static void process(uint8_t) {
//        std::cout << "hbb"_pgm << std::endl;
        crWriterSensorBinary::enable<true>();
        crWriterSensorText::enable<false>();
        crAdapterHott::start();
    }
};

class HottKeyHandler : public EventHandler<EventType::HottAsciiKey> {
public:
    static void process(uint8_t v) {
        std::cout << "k: "_pgm << v << std::endl;
//        Hott::SensorProtocoll<sensorUsart>::key(v);

    }
};

class HottBroadcastHandler : public EventHandler<EventType::HottSensorBroadcast> {
public:
    static void process(uint8_t) {
        std::cout << "hbr"_pgm << std::endl;
        crWriterSensorBinary::enable<true>();
        crWriterSensorText::enable<false>();
        crAdapterHott::start();
    }
};

class HottTextHandler : public EventHandler<EventType::HottAsciiRequest> {
public:
    static void process(uint8_t) {
        std::cout << "hba"_pgm << std::endl;
        crWriterSensorBinary::enable<false>();
        crWriterSensorText::enable<true>();
        crAdapterHott::start();
    }
};

decltype(systemTimer::create(1000_ms, TimerFlags::Periodic)) pTimer;
decltype(systemTimer::create(1000_ms, TimerFlags::Periodic)) mTimer;
decltype(systemTimer::create(1000_ms, TimerFlags::Periodic)) tTimer;

class TimerHandler : public EventHandler<EventType::Timer> {
public:
    static void process(uint8_t timer) {
        if (timer == *pTimer) {
            static uint8_t count = 0;
            WS2812<2, ws2812_A>::set({16, (uint8_t)((count++ % 2) * 16), 16});
            
            ds1307::startReadTimeInfo();
                      
            std::cout << "Temp lm35: "_pgm << lm35::temperature() << std::endl;
            std::cout << "ppm:"_pgm << ppm1::value() << std::endl;
            std::cout << "c0: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(0) << std::endl;
    //        std::cout << "c1: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(1) << std::endl;
    //        std::cout << "c2: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(2) << std::endl;
    //        std::cout << "c3: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(3) << std::endl;
    //        std::cout << "c4: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(4) << std::endl;
    //        std::cout << "c5: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(5) << std::endl;
    //        std::cout << "nn: "_pgm << Hott::SumDProtocollAdapter<0>::numberOfChannels() << std::endl;
    
            std::percent pv = std::scale(Hott::SumDProtocollAdapter<0>::value8Bit(0),
                                   Hott::SumDMsg::Low8Bit, Hott::SumDMsg::High8Bit);
            std::cout << "pv: " << pv.value << std::endl;
            softPpm::ppm(pv, 0);
    
            softPwm::pwm(pv, 0);
            softPwm::pwm(pv, 1);
            std::cout << "spwm period: "_pgm << softPwm::period() << std::endl;
        }
        else if (timer == *tTimer) {
            if (ds18b20::convert()) {
                systemTimer::start(*mTimer);
            }
        }
        else if (timer == *mTimer) {
            ds18b20::startGet(dsIds[0]);
        }
    }
};

class TestHandler : public EventHandler<EventType::Test> {
public:
    static void process(uint8_t) {
        std::cout << "t"_pgm << std::endl;
    }
};

class PpmUpHandler : public EventHandler<EventType::Ppm1Up> {
public:
    static void process(uint8_t) {
        std::cout << "ppm1up"_pgm << std::endl;
    }
};
class PpmDownHandler : public EventHandler<EventType::Ppm1Down> {
public:
    static void process(uint8_t) {
        std::cout << "ppm1down"_pgm << std::endl;
    }
};
class UsartHandler : public EventHandler<EventType::UsartRecv0> {
public:
    static void process(uint8_t v) {
        std::cout << "u: "_pgm << v << std::endl;
    }
};

using isrRegistrar = IsrRegistrar<ppm1, isrDistributor>;

int main()
{
    isrRegistrar::init();
    Scoped<EnableInterrupt> interruptEnabler;

    systemTimer::init();
//    terminal::init<0>();
    bufferedTerminal::init<0>();

    sensorUsart::init<19200>();
    rcUsart::init<115200>();
    SSpi0::init();
    buttonController::init();
    adcController::init();
    
    using namespace std::literals::quantity;
    softPpm::init();
    softPpm::ppm(50_ppc, 0);

    hardPwm::init();
    hardPwm::pwm<hardPwm::A>(90_ppc);
    hardPwm::pwm<hardPwm::B>(50_ppc);

//    testPin::dir<AVR::Output>();

    ds1307::init();
    ds1307::squareWave<true>();
    
    dcfDecoder::init();

    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    constexpr auto tsd = AVR::Util::calculate<crTimer>(fCr);
    crTimer::prescale<tsd.prescaler>();
    crTimer::ocra<tsd.ocr>();

    crAdapterHott::init();
    crAdapterOneWire::init();
    
    ds18b20::init();

    oneWireMaster::findDevices(dsIds);
    for(const auto& id : dsIds) {
        std::cout << id << std::endl;
    }
            
    led::dir<AVR::Output>();

    pinChangeHandlerPpm::init();
    PpmDecoder<pinChangeHandlerPpm, ppmTimerInput>::init();

    std::cout << "RC Controller 0.1"_pgm << std::endl;

    WS2812<2, ws2812_A>::init();
    WS2812<2, ws2812_A>::off();

//    WS2812<2, ws2812_B>::init();
//    WS2812<2, ws2812_B>::off();

    pTimer = systemTimer::create(1000_ms, TimerFlags::Periodic);
    tTimer = systemTimer::create(5000_ms, TimerFlags::Periodic);
    mTimer = systemTimer::create(750_ms, TimerFlags::OneShot);
    systemTimer::stop(*mTimer);
    
//    std::cout << Config() << std::endl;

    using handler = EventHandlerGroup<TimerHandler,
                                HottBinaryHandler, HottBroadcastHandler, HottTextHandler, TestHandler,
                                PpmDownHandler, PpmUpHandler,
                                UsartHandler, HottKeyHandler,
                                Button0Handler, 
                                ds18b20, DS18B20ErrorHandler, DS18B20MeasurementHandler,
                                TWIHandlerError, ds1307, DS1307handler, DS1307handlerError>;

    EventManager::run<sampler, handler>([](){
        led::toggle();
        ppmSwitch::process(ppm1::value());
        softPwm::freeRun();
        crAdapterHott::periodic();
        vrAdapter::periodic();
        crAdapterOneWire::periodic();
        TwiMasterAsync::periodic();
    });

    return 0;
}

#ifndef NDEBUG
constexpr void assertFunction(bool b, const char* function, const char* file, unsigned int line) {
   if (!b) {
        std::cout << "Assertion failed: "_pgm << function << ","_pgm << file << ","_pgm << line << std::endl;
        abort();
    }
}
#endif

ISR(PCINT0_vect) {

}
ISR(PCINT1_vect) {

}
ISR(PCINT2_vect) {
    isrRegistrar::isr<AVR::ISR::PcInt<2>>();
//    ppm1::isr();
}
ISR(PCINT3_vect) {
}
ISR(TIMER1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
//    crAdapter::rateTick();
//    led::toggle();
//    SWUsart<0>::isr_compa();
}
ISR(TIMER1_COMPB_vect) {
//    SWUsart<0>::isr_compb();
}
ISR(TIMER1_CAPT_vect) {
//    SWUsart<0>::isr_icp();
}
ISR(SPI_STC_vect) {
    AVR::Spi<0>::isr();
}
ISR(TIMER0_COMPA_vect) {
    // todo: auf isrRegistrar umbauen
//    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
    sampler::tick();
}
ISR(TIMER3_COMPA_vect) {
    softPpm::isrA();
}
ISR(TIMER3_COMPB_vect) {
    softPpm::isrB();
}
ISR(USART0_RX_vect) {
    sensorUsart::rx_isr();
}
ISR(USART0_UDRE_vect){
    sensorUsart::tx_isr();
}
ISR(USART1_RX_vect) {
    rcUsart::rx_isr();
}
ISR(USART1_UDRE_vect){
    rcUsart::tx_isr();
}

