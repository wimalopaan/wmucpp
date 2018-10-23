#define NDEBUG

#include "local.h"
#include "varioprop.h"

#include <console.h>

namespace Constants {
    static constexpr std::hertz pwmFrequency = 100_Hz * 256; 
    static constexpr const std::hertz fSCL = 100000_Hz;
    static constexpr Color cRed{Red{32}};
    static constexpr Color cBlue{Blue{32}};
    static constexpr Color cGreen{Green{32}};
    static constexpr Color cOff{};
}

using terminalDevice = DebugUsart;
using terminal = std::basic_ostream<terminalDevice>;

using bt = std::basic_ostream<BTUsart>;

using logger = RoboRemo::Logging<bt, 0>;
using plot0  = RoboRemo::ValuePlot<bt, 0>;

template<typename A>
class Controls {
    using adc = A;
public:
    using value_type = typename A::value_type;
    using channels_type = uint_ranged<uint8_t, 0, adc::NumberOfChannels - 1>;
    
    inline static void init() {
        adc::init();
    }
    inline static void periodic() {
        adc::periodic();    
    }
    inline static value_type value(channels_type channel) {
        return adc::value(channel);    
    }
    template<uint8_t Lower, uint8_t Upper>
    inline static value_type value(uint_ranged<uint8_t, Lower, Upper> channel) {
        static_assert(Lower >= 0);
        static_assert(Upper <= (adc::NumberOfChannels - 1));
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

int main() {
    isrRegistrar::init();
    display::init();
    controls::init();
    DebugUsart::init<9600>();
    BTUsart::init<9600>();
    TeleUsart::init<9600>();
    hardPwm::init<Constants::pwmFrequency>();
   
    cppm::init();
    
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 

    const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);
    
    {
        display::blinkStart();

        hardPwm1::pwm(uint8_t{128});
        hardPwm2::pwm(uint8_t{128});
        
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>("Test05"_pgm);
        
        auto channel = uint_ranged_circular<uint8_t, 0, 3>{};
        while(true) {
            controls::periodic();
            cppm::periodic();
            cppm::ppm(channel.toRanged(), controls::value(channel.toRanged()));
            ++channel;
            cppm::ppm(cppm::index_type{0}, uint_ranged<uint8_t, 0, 255>{RoboRemoPA::propValues[0]});
            systemClock::periodic<systemClock::flags_type::ocfa>([&](){
                alarmTimer::periodic([&](uint7_t timer) {
                    static uint_ranged_circular<uint8_t, 0, adcController::NumberOfChannels - 1> counter;
                    if (timer == *periodicTimer)  {
                         display::tick();
                         std::outl<terminal>("A: "_pgm, counter.toInt(), " : "_pgm, controls::value(counter.toRanged()).toInt());
                         std::outl<terminal>("ta1: "_pgm, FrSkyPA::data[0], " ta2: "_pgm, FrSkyPA::data[1]);
                         std::outl<terminal>("rb0:"_pgm, RoboRemoPA::propValues[0]);
                         ++counter;
                         logger::outl("test05: "_pgm, counter.toInt());
                         plot0::put((uint8_t)FrSkyPA::data[1]);
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
