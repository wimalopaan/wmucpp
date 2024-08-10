#define NDEBUG

// use the following options exclusive
//#define USE_GPIO_AS_GND
#define USE_LED

#include <mcu/avr.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <etl/output.h>
#include <etl/meta.h>

#include "hw.h"

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace {
    static constexpr auto fRtc = 1000_Hz;
    // static constexpr uint8_t controllerNumber = 0x00; // 0x00: internal, unconditional sending
    static constexpr uint8_t controllerNumber = 0x01; // 0x01: external, waiting for message from controller 0x00
    // static constexpr uint8_t controllerNumber = 0x02; // 0x02: external, waiting for message from controller 0x01
}

using s11 = ActiveLow<Pin<Port<A>, 4>, Input>; // Pin 2
using s12 = ActiveLow<Pin<Port<A>, 5>, Input>; // Pin 3
using t1  = ActiveLow<Pin<Port<A>, 6>, Input>; // Pin 4
using s2  = ActiveLow<Pin<Port<B>, 3>, Input>; // Pin 6
using t2  = ActiveLow<Pin<Port<B>, 2>, Input>; // Pin 7

#ifdef USE_GPIO_AS_GND
using o1 = ActiveLow<Pin<Port<A>, 7>, Output>; // Pin 5
using o2 = ActiveLow<Pin<Port<B>, 0>, Output>; // Pin 9
using o3 = ActiveLow<Pin<Port<A>, 2>, Output>; // Pin 12 (rx: unused)
using o4 = ActiveLow<Pin<Port<A>, 3>, Output>; // Pin 13
using o5 = ActiveLow<Pin<Port<B>, 1>, Output>; // Pin 8
#endif

#ifdef USE_LED
using led = ActiveHigh<Pin<Port<B>, 1>, Output>;
#endif

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
using serialout = External::HW::Output::Generator<usart0Position, systemTimer, controllerNumber>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

struct FSM final {
    static inline constexpr External::Tick<systemTimer> initTicks{300_ms};

    enum class State : uint8_t {Undefined, Run};

    static inline void init() {
#ifdef USE_GPIO_AS_GND
        o1::init(); // use some outputs as "low" for the switches ...
        o2::init(); // ... just to make to wiring easier ;-)
        o3::init();
        o4::init();
        o5::init();

        o1::activate();
        o2::activate();
        o3::activate();
        o4::activate();
        o5::activate();
#endif
#ifdef USE_LED
        led::init();
#endif
        s11::init();
        s12::init();
        t1::init();
        s2::init();
        t2::init();
        serialout::init();
    }
    static inline void periodic() {
        serialout::periodic();
    }
    static inline void ratePeriodic() {
        serialout::ratePeriodic();
        ++mStateTicks;
        switch(mState) {
        case State::Undefined:
            mStateTicks.on(initTicks, []{
                mState = State::Run;
            });
            break;
        case State::Run:
            update();
#ifdef USE_LED
            if (serialout::fail()) {
                led::inactivate();
            }
            else {
                led::activate();
            }
#endif
            break;
        }
    }
private:
    static inline void update() {
        uint8_t b{0x00};
        if (s11::isActive()) {
            b |= 0x01;
        }
        if (s12::isActive()) {
            b |= 0x02;
        }
        if (t1::isActive()) {
            b |= 0x04;
        }
        if (s2::isActive()) {
            b |= 0x08;
        }
        if (t2::isActive()) {
            b |= 0x10;
        }
        serialout::set(b);
    }
    inline static State mState{State::Undefined};
    static inline External::Tick<systemTimer> mStateTicks;
    static inline External::Tick<systemTimer> mEepromTicks;
};

int main() {
    using fsm = FSM;
    portmux::init();
    ccp::unlock([]{
        clock::prescale<1>();
    });
    systemTimer::init();

    fsm::init();
    while(true) {
        fsm::periodic();
        systemTimer::periodic([&]{
            fsm::ratePeriodic();
        });
    }
}

