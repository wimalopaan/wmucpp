#include "main.h"
#include "limits.h"
#include "mcu.h"
#include "mcutimer.h"
#include "usart.h"
#include "event.h"
#include "ports.h"
#include "physical.h"
#include "softtimer.h"
#include "literals.h"
#include "isr.h"
#include "spi.h"
#include "swusart.h"
#include "ws2812.h"
#include "pinchange.h"
#include "ppm.h"
#include "ppmswitch.h"
#include "fsm.h"
#include "hott.h"
#include "delay.h"
#include "console.h"
#include "softspimaster.h"
#include "button.h"
#include "dcf77.h"
#include "softppm.h"
#include "mcupwm.h"

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
using ppmTimer = AVR::Timer8Bit<2>;
using ppm1 = PpmDecoder<pinChangeHandlerPpm, ppmTimer>;
using ppmSwitch = PpmSwitch<0, ppm1>;

using systemClock = AVR::Timer8Bit<0>;
using systemTimer = Timer<systemClock>;

using sensorUsart = AVR::Usart<0, Hott::SensorProtocollAdapter<0>> ;
using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>>;

using sampler = PeriodicGroup<buttonController, systemTimer, dcfDecoder>;

using pwmTimer = AVR::Timer16Bit<3>;
//using pwmPin = AVR::Pin<PortD, 6>;
//using softPwm = SoftPWM<pwmTimer, pwmPin>;

using hardPwm = AVR::PWM<pwmTimer>;

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
        Hott::SensorProtocoll<sensorUsart>::hott_response();
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
        Hott::SensorProtocoll<sensorUsart>::hott_response();
    }
};

class HottTextHandler : public EventHandler<EventType::HottAsciiRequest> {
public:
    static void process(const uint8_t&) {
        std::cout << "hba"_pgm << std::endl;
        Hott::SensorProtocoll<sensorUsart>::hott_responseAscii();

    }
};

class TimerHandler : public EventHandler<EventType::Timer> {
public:
    static void process(const uint8_t&) {
        static uint8_t count = 0;
        WS2812<2, ws2812_A>::set({16, (uint8_t)((count++ % 2) * 16), 16});
        std::cout << "ppm:"_pgm << ppm1::value() << std::endl;
        std::cout << "c0: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(0) << std::endl;
        std::cout << "c1: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(1) << std::endl;
        std::cout << "c2: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(2) << std::endl;
        std::cout << "c3: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(3) << std::endl;
        std::cout << "c4: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(4) << std::endl;
        std::cout << "c5: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(5) << std::endl;
        std::cout << "nn: "_pgm << Hott::SumDProtocollAdapter<0>::numberOfChannels() << std::endl;

        std::percent pv = std::scale(Hott::SumDProtocollAdapter<0>::value8Bit(0),
                               Hott::SumDMsg::Low8Bit, Hott::SumDMsg::High8Bit);
//        softPwm::pwm(pv, 0);
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

    using namespace std::literals::percent;
//    softPwm::init();
//    softPwm::pwm(50_ppc, 0);
    hardPwm::init();
    hardPwm::pwm<hardPwm::A>(50_ppc);

    dcfDecoder::init();

    led::dir<AVR::Output>();

    pinChangeHandlerPpm::init();
    PpmDecoder<pinChangeHandlerPpm, ppmTimer>::init();

    std::cout << "RC Controller 0.1"_pgm << std::endl;

    WS2812<2, ws2812_A>::init();
    WS2812<2, ws2812_A>::off();

    WS2812<2, ws2812_B>::init();
    WS2812<2, ws2812_B>::off();

    systemTimer::create(1000_ms, TimerFlags::Periodic);

    std::cout << Config() << std::endl;

    std::cout << "ppmtimer: "_pgm << ppm1::timerFrequency << std::endl;
    std::cout << "prescaler: "_pgm << ppm1::prescaler << std::endl;
    std::cout << "ppmMin: "_pgm << ppm1::ppmMin << std::endl;
    std::cout << "ppmMax: "_pgm << ppm1::ppmMax << std::endl;
    std::cout << "ppmMid: "_pgm << ppm1::ppmMid << std::endl;
    std::cout << "ppmDelta: "_pgm << ppm1::ppmDelta << std::endl;
    std::cout << "ppmMidLow: "_pgm << ppm1::ppmMidLow << std::endl;
    std::cout << "ppmMidHigh: "_pgm << ppm1::ppmMidHigh << std::endl;
    std::cout << "ppmMaxLow: "_pgm << ppm1::ppmMaxLow << std::endl;
    std::cout << "ppmMinHigh: "_pgm << ppm1::ppmMinHigh << std::endl;

//    std::cout << "pwm pre: "_pgm << softPwm::prescaler << std::endl;

    using handler = EventHandlerGroup<TimerHandler,
                                HottBinaryHandler, HottBroadcastHandler, HottTextHandler, TestHandler,
                                PpmDownHandler, PpmUpHandler,
                                UsartHandler, HottKeyHandler,
                                Button0Handler>;

    EventManager::run<sampler, handler>([](){
        led::toggle();
        ppmSwitch::process(ppm1::value());
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
    SWUsart<0>::isr_compa();
}
ISR(TIMER1_COMPB_vect) {
    SWUsart<0>::isr_compb();
}
ISR(TIMER1_CAPT_vect) {
    SWUsart<0>::isr_icp();
}
ISR(SPI_STC_vect) {
    AVR::Spi<0>::isr();
}
ISR(TIMER0_COMPA_vect) {
    ++sampler::tickCounter;
}
ISR(TIMER3_COMPA_vect) {
//    softPwm::isrA();
}
ISR(TIMER3_COMPB_vect) {
//    softPwm::isrB();
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

