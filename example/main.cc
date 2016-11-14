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
#include "hal/ppmswitch.h"
#include "util/fsm.h"
#include "external/hott/hott.h"
#include "util/delay.h"
#include "console.h"
#include "hal/softspimaster.h"
#include "hal/button.h"
#include "external/dcf77.h"
#include "hal/softppm.h"
#include "mcu/avr/mcupwm.h"
#include "hal/softpwm.h"
#include "hal/constantrate.h"

#include <stdlib.h>

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

using terminal = SSpi0;

namespace std {
    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

using button0 = Button<0, AVR::Pin<PortA, 3>>;
using button1 = Button<1, AVR::Pin<PortA, 4>>;
using buttonController = ButtonController<button0, button1>;

using dcfPin = AVR::Pin<PortA, 7>;
using dcfDecoder = DCF77<dcfPin>;

using ws2812_A = AVR::Pin<PortC, 1>;
using ws2812_B = AVR::Pin<PortC, 2>;

using ppmInputPin = AVR::Pin<PortC, 0>;
using pinChangeHandlerPpm = AVR::PinChange<ppmInputPin>;
using ppmTimerInput = AVR::Timer8Bit<2>;
using ppm1 = PpmDecoder<pinChangeHandlerPpm, ppmTimerInput>;
using ppmSwitch = PpmSwitch<0, ppm1>;

using systemClock = AVR::Timer8Bit<0>;
using systemTimer = Timer<systemClock>;

using sensorUsart = AVR::Usart<0, Hott::SensorProtocollAdapter<0>> ;
using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>>;


using ppmTimerOutput = AVR::Timer16Bit<3>;
using ppmPin1 = AVR::Pin<PortD, 7>;
using ppmPin2 = AVR::Pin<PortD, 6>;
using softPpm = SoftPPM<ppmTimerOutput, ppmPin1>;

// todo: auf pwm verzichten, dafür ConstantRateAdapter mit Timer 1
//using hardPwm = AVR::PWM<1>;


using crTestPin = AVR::Pin<PortB, 2>;
using crTimer = AVR::Timer16Bit<1>;
using crWriter = ConstanteRateWriter<Hott::SensorProtocollBuffer<0>, sensorUsart>;
using crAdapter = ConstantRateAdapter<crTimer, crWriter, TestBitShifter<crTestPin, 0x55>>;


using softPwmPin1 = AVR::Pin<PortB, 1>;
using softPwm = SoftPWM<softPwmPin1>;

using sampler = PeriodicGroup<buttonController, systemTimer, dcfDecoder, softPwm>; // werden alle resolution ms aufgerufen

//using testPin = AVR::Pin<PortD, 5>;

using led = AVR::Pin<PortB, 0>;
//using led = AVR::Pin<PortB, 2>;
//using led2 = AVR::Pin<PortC, 3>;

struct EventHandlerParameter {
    std::optional<uint7_t> timerId1;
};

class Button0Handler: public EventHandler<EventType::ButtonPress0> {
public:
    static void process(const uint8_t&) {
        std::cout << "button 0 press"_pgm << std::endl;
    }
};

class HottBinaryHandler : public EventHandler<EventType::HottBinaryRequest> {
public:
    static void process(const uint8_t&) {
//        std::cout << "hbb"_pgm << std::endl;
        crAdapter::start();
    }
};

class HottKeyHandler : public EventHandler<EventType::HottAsciiKey> {
public:
    static void process(const uint8_t& v) {
        std::cout << "k: "_pgm << v << std::endl;
        Hott::SensorProtocoll<sensorUsart>::key(v);

    }
};

class HottBroadcastHandler : public EventHandler<EventType::HottSensorBroadcast> {
public:
    static void process(const uint8_t&) {
        std::cout << "hbr"_pgm << std::endl;
        crAdapter::start();
    }
};

class HottTextHandler : public EventHandler<EventType::HottAsciiRequest> {
public:
    static void process(const uint8_t&) {
        std::cout << "hba"_pgm << std::endl;
//        Hott::SensorProtocoll<sensorUsart>::hott_responseAscii();
    }
};

class TimerHandler : public EventHandler<EventType::Timer> {
public:
    static void process(const uint8_t&) {
        static uint8_t count = 0;
        WS2812<2, ws2812_A>::set({16, (uint8_t)((count++ % 2) * 16), 16});
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
        
        std::cout << "spwm period: "_pgm << softPwm::period() << std::endl;

    }
};

class TestHandler : public EventHandler<EventType::Test> {
public:
    static void process(const uint8_t&) {
        std::cout << "t"_pgm << std::endl;
    }
};

class PpmUpHandler : public EventHandler<EventType::Ppm1Up> {
public:
    static void process(const uint8_t&) {
        std::cout << "ppm1up"_pgm << std::endl;
    }
};
class PpmDownHandler : public EventHandler<EventType::Ppm1Down> {
public:
    static void process(const uint8_t&) {
        std::cout << "ppm1down"_pgm << std::endl;
    }
};
class UsartHandler : public EventHandler<EventType::UsartRecv0> {
public:
    static void process(const uint8_t& v) {
        std::cout << "u: "_pgm << v << std::endl;
    }
};

int main()
{
    Scoped<EnableInterrupt> interruptEnabler;

    systemTimer::init();
    terminal::init<0>();

    sensorUsart::init<19200>();
    rcUsart::init<115200>();
    SSpi0::init();
    buttonController::init();

    using namespace std::literals::quantity;
    softPpm::init();
    softPpm::ppm(50_ppc, 0);

//    hardPwm::init();
//    hardPwm::pwm<hardPwm::A>(90_ppc);
//    hardPwm::pwm<hardPwm::B>(50_ppc);

//    testPin::dir<AVR::Output>();

    dcfDecoder::init();

    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    constexpr auto tsd = AVR::Util::calculate<crTimer>(fCr);
    crTimer::prescale<tsd.prescaler>();
    crTimer::mcuTimer->ocra = tsd.ocr;
    crAdapter::init();
    
    
    led::dir<AVR::Output>();

    pinChangeHandlerPpm::init();
    PpmDecoder<pinChangeHandlerPpm, ppmTimerInput>::init();

    std::cout << "RC Controller 0.1"_pgm << std::endl;

    WS2812<2, ws2812_A>::init();
    WS2812<2, ws2812_A>::off();

    WS2812<2, ws2812_B>::init();
    WS2812<2, ws2812_B>::off();

    systemTimer::create(1000_ms, TimerFlags::Periodic);

//    std::cout << Config() << std::endl;

    using handler = EventHandlerGroup<TimerHandler,
                                HottBinaryHandler, HottBroadcastHandler, HottTextHandler, TestHandler,
                                PpmDownHandler, PpmUpHandler,
                                UsartHandler, HottKeyHandler,
                                Button0Handler>;

    EventManager::run<sampler, handler>([](){
//        led::toggle();
        ppmSwitch::process(ppm1::value());
        softPwm::freeRun();
        crAdapter::periodic();
    });

    return 0;
}

void assertFunction(bool b, const char* function, const char* file, unsigned int line) {
    if (!b) {
        std::cout << "Assertion failed: " << function << "," << file << "," << line << std::endl;
        abort();
    }
}

ISR(PCINT0_vect) {

}
ISR(PCINT1_vect) {

}
ISR(PCINT2_vect) {
    ppm1::isr2();
}
ISR(PCINT3_vect) {
}
ISR(TIMER1_COMPA_vect) {
    crAdapter::rateTick();
    led::toggle();
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

