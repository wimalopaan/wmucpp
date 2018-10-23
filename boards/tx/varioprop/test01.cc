#define NDEBUG

#include "local.h"
#include "varioprop.h"

namespace Constants {
    static constexpr std::hertz pwmFrequency = 100_Hz * 256; 
    static constexpr const std::hertz fSCL = 100000_Hz;
    static constexpr Color cRed{Red{32}};
    static constexpr Color cBlue{Blue{32}};
    static constexpr Color cGreen{Green{32}};
    static constexpr Color cOff{};
}

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

int main() {
    isrRegistrar::init();
    display::init();

    alarmTimer::init(AVR::TimerMode::CTCNoInt); 

    const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);
    
    {
        display::blinkStart();
        
        Scoped<EnableInterrupt<>> ei;
        while(true) {
            systemClock::periodic<systemClock::flags_type::ocfa>([&](){
                alarmTimer::periodic([&](uint7_t timer){
                    if (timer == *periodicTimer)  {
                         display::tick();
                    }
                });
            
            });
        
        }
    }
}

ISR(USART2_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<2>::RX>();
}
ISR(USART2_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<2>::UDREmpty>();
}
ISR(USART1_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
}
ISR(USART1_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
}
ISR(USART0_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART0_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
}
#endif
