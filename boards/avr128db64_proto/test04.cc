#include "devices.h"

// Rotary + DAC(pd6 = A_Is) + VRef (VRef)

template<typename Devices>
struct GlobalFsm {
    using devs = Devices;
    using blink1_t = devs::blinkLed1::count_type;
    using blink2_t = devs::blinkLed2::count_type;
    
    enum class State : uint8_t {Undefined, On, Blink1, Blink2, BlinkOff};

    static inline constexpr External::Tick<typename devs::systemTimer> powerTicks{1000_ms};
    
    static void init() {
        devs::init();
        
        devs::blinkLed1::init();
        devs::blinkLed2::init();

        devs::rotary::init(127);
        devs::rotaryButton::init();
        
        devs::dac::init();
    } 
    static void periodic() {
        devs::la0::toggle();
    } 
    static void ratePeriodic() {
        devs::la1::toggle();        
        devs::rotary::rateProcess();
        devs::rotaryButton::ratePeriodic();
        devs::blinkLed1::ratePeriodic();
        devs::blinkLed2::ratePeriodic();
        
        const auto oldState{mState};
        ++mStateTicks;
        
        const uint16_t r = devs::rotary::value();
        devs::dac::put(r);
        
        switch(mState) {
        case State::Undefined:
            mStateTicks.on(powerTicks, []{
                mState = State::On;
            });
            break;
        case State::On:
            break;
        case State::Blink1:
            if (const auto e = devs::rotaryButton::event(); e == devs::rotaryButton::Press::Short) {
                mState = State::Blink2;
            }
            else if (e == devs::rotaryButton::Press::Long) {
                mState = State::BlinkOff;
            }
            break;
        case State::Blink2:
            if (const auto e = devs::rotaryButton::event(); e == devs::rotaryButton::Press::Short) {
                mState = State::Blink1;
            }
            else if (e == devs::rotaryButton::Press::Long) {
                mState = State::BlinkOff;
            }
            break;
        case State::BlinkOff:
            if (const auto e = devs::rotaryButton::event(); e == devs::rotaryButton::Press::Long) {
                mState = State::Blink1;
            }
            break;
        }
        if (oldState != mState) {
            mStateTicks.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::On:
                devs::blinkLed1::blink(blink1_t{4});
                break;
            case State::Blink1:
                devs::blinkLed2::blink(blink2_t{2});
                break;
            case State::Blink2:
                devs::blinkLed2::blink(blink2_t{1});
                break;
            case State::BlinkOff:
                devs::blinkLed2::off();
                break;
            }
        }
    }     
private:    
    static inline State mState{State::Undefined};
    static inline External::Tick<typename devs::systemTimer> mStateTicks;
};

using devices = Devices<>;
using gfsm = GlobalFsm<devices>;

int main() {
    gfsm::init();    
    while(true) {
        gfsm::periodic(); 
        devices::systemTimer::periodic([&]{
            gfsm::ratePeriodic();
        });
    }
    
}
