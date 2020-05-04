#include "board.h"
#include "temp.h"
#include "curr.h"
#include "rpm.h"
#include "gps.h"

using adcController = External::Hal::AdcController<adc, Meta::NList<10, 11>>; // 0x1e = temp

using curr0P = ACS723U40Provider<adcController, 0>;
using curr1P = ACS723U40Provider<adcController, 1>;

template<typename MCU = DefaultMcuType>
struct FSM {
    enum class State : uint8_t {Init, Calibrate, Run};
    
    static inline constexpr uint8_t initCount = 30; // 3s
    static inline constexpr uint8_t caliCount = 50;
    
    static inline void init() {
    }
    static inline void tick() {
        ++mTickCount;
        switch(mState) {
        case State::Init:
            if (mTickCount > initCount) {
                mState = State::Calibrate;
                mTickCount = 0;
            }
            break;
        case State::Calibrate:
            curr0P::accumulateCalibration();
            curr1P::accumulateCalibration();
            if (mTickCount > caliCount) {
                mState = State::Run;
                mTickCount = 0;
                curr0P::setCalibration();
                curr1P::setCalibration();
            }
            break;
        case State::Run:
            break;
        }
    }
//private:
    inline static uint8_t mTickCount{};
    inline static State mState{State::Init};
};
using fsm = FSM<>;

template<typename FSM>
struct StateProvider {
    inline static constexpr auto ibus_type = IBus::Type::type::ARMED; 
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
        return uint16_t(FSM::mState);
    }
};

using stateP = StateProvider<fsm>;



using ibus = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, 
                          Meta::List<curr0P, curr1P, rpm0P, rpm1P, speedP, 
                          bytesRP, bytesVP, stateP>, 
                          systemTimer, ibt>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

using evrouter = Event::Router<Event::Channels<evch0, evch1>, Event::Routes<evuser0, evuser1>>;
using isrRegistrar = IsrRegistrar<typename gpsUsart::StartBitHandler, typename gpsUsart::BitHandler>;

int main() {
    wdt::init<ccp>();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    reset::noWatchDog([]{
        uninitialzed::reset();
    });
    reset::onWatchDog([]{
        uninitialzed::counter = uninitialzed::counter + 1;        
    });
    
    fsm::init();
    
    evrouter::init();
    portmux::init();
    systemTimer::init();
    adcController::init();
    
    ibus::init();
    gpsUsart::init();
    
    rpm0::init();   
    rpm1::init();   
    
    const auto periodicTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);
    const auto resetTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    
    while(true) {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        ibus::periodic();
        adcController::periodic();
        
        systemTimer::periodic([&]{
            wdt::reset();
            ibus::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    fsm::tick();
                }
                else if (resetTimer == t) {
                    rpm0::reset();
                    rpm1::reset();
                }
            });
        });
    }
}

ISR(PORTA_PORT_vect) {
    isrRegistrar::isr<AVR::ISR::Port<rxPin::name_type>>();
}

ISR(TCD0_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Tcd<0>::Ovf>();
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
#if !(defined(USE_IBUS) || defined(USE_HOTT))
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
#endif
    while(true) {
        dbg1::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
