#define NDEBUG

#include "local.h"
#include "varioprop.h"

#include <console.h>

using p2 = AVR::Pin<PortD, 7>;


namespace Constants {
    static constexpr std::hertz pwmFrequency = 2000_Hz * 256; 
    static constexpr const std::hertz fSCL = 100000_Hz;
    static constexpr Color cRed{Red{32}};
    static constexpr Color cBlue{Blue{32}};
    static constexpr Color cGreen{Green{32}};
    static constexpr Color cOff{};
}

using terminalDevice = DebugUsart;
using terminal = std::basic_ostream<terminalDevice>;

template<typename A>
class Controls {
    using adc = A;
public:
    using value_type = typename A::value_type;
    using channels_type = uint_ranged_circular<uint8_t, 0, adc::NumberOfChannels - 1>;
    
    inline static void init() {
        adc::init();
    }
    inline static void periodic() {
        adc::periodic();    
    }
    inline static value_type value(channels_type channel) {
        return adc::value(channel);    
    }
private:
};

using controls = Controls<adcController>;

template<typename L>
class LedDisplay {
    using Led = L;
    enum class State : uint8_t {Off, BlinkOff, BlinkOn};
public:
    inline static void init() {
        Led::init();
        Led::off();
    }
    inline static void blinkStart() {
        mState = State::BlinkOn;
    }
    inline static void tick() {
        switch (mState) {
        case State::Off:
            break;
        case State::BlinkOff:
            mState = State::BlinkOn;
            led::set(Constants::cGreen);
            break;
        case State::BlinkOn:
            mState = State::BlinkOff;
            Led::off();
            break;
        default:
            break;
        }
    }
private:
    inline static State mState = State::Off;
};

using display = LedDisplay<led>;

using isrRegistrar = IsrRegistrar<BTUsart::RxHandler, BTUsart::TxHandler, TeleUsart::RxHandler, TeleUsart::TxHandler, DebugUsart::RxHandler, DebugUsart::TxHandler>;

//    isrRegistrar::init();
    
//    DebugUsart::init<9600>();
    
//    display::init();
//    controls::init();
    
//    hardPwm::init<Constants::pwmFrequency>();
    
//    UCSR2B = 0x10; // oder 0x08
int main() {
    UCSR2B = _BV(RXEN2);

    TCCR2A = 0xf3;
    TCCR2B = 0x03;
    OCR2A = 128;
    OCR2B = 128;
    DDRD |= _BV(6) | _BV(7);
    
    while(true);
    
#if 0
    
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 

    const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);
    
    {
//        hardPwm1::pwm(uint8_t{128});
//        hardPwm2::pwm(uint8_t{128});
        
        display::blinkStart();
        
        Scoped<EnableInterrupt<>> ei;
        while(true) {
            controls::periodic();
            systemClock::periodic<systemClock::flags_type::ocfa>([&](){
                alarmTimer::periodic([&](uint7_t timer) {
                    static uint_ranged_circular<uint8_t, 0, adcController::NumberOfChannels - 1> counter;
                    if (timer == *periodicTimer)  {
                         display::tick();
                         
                         std::outl<terminal>("A: "_pgm, (uint8_t)counter, " : "_pgm, controls::value(counter).toInt());
                         ++counter;
                         
//                         hardPwm1::pwm(controls::value(uint_ranged_circular<uint8_t, 0, 6>{0}).toInt());
                    }
                });
            });
        }
    }
#endif
}

//ISR(USART2_RX_vect) {
//    isrRegistrar::isr<AVR::ISR::Usart<2>::RX>();
//}
//ISR(USART2_UDRE_vect){
//    isrRegistrar::isr<AVR::ISR::Usart<2>::UDREmpty>();
//}
//ISR(USART1_RX_vect) {
//    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
//}
//ISR(USART1_UDRE_vect){
//    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
//}
//ISR(USART0_RX_vect) {
//    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
//}
//ISR(USART0_UDRE_vect){
//    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
//}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
}
#endif
