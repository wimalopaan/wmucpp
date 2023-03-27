#include "devices.h"

template<typename Devices>
struct GlobalFsm {
    using devs = Devices;
    using blink1_t = devs::blinkLed1::count_type;
    using blink2_t = devs::blinkLed2::count_type;
    
    enum class State : uint8_t {Undefined, PreOn, On, PreOff, Off};

    static inline constexpr External::Tick<typename devs::systemTimer> powerTicks{1000_ms};
    
    static void init() {
        devs::init();
        
        devs::blinkLed1::init();
        devs::blinkLed2::init();

        devs::blinkLed1::blink(blink1_t{4});
        
        devs::powerSwitch::init();
    } 
    static void periodic() {
        devs::la0::toggle();
        
    } 
    static void ratePeriodic() {
        devs::la1::toggle();        
        devs::blinkLed1::ratePeriodic();
        devs::blinkLed2::ratePeriodic();
    
        const auto oldState{mState};
        ++mStateTicks;
        
        switch(mState) {
        case State::Undefined:
            mStateTicks.on(powerTicks, []{
                mState = State::PreOn;
            });
            break;
        case State::PreOn:
            mStateTicks.on(powerTicks, []{
                mState = State::On;
            });
            break;
        case State::On:
            if (devs::powerSwitch::isActive()) {
                mState = State::PreOff;
            }
            break;
        case State::PreOff:
            mStateTicks.on(powerTicks, []{
                if (devs::powerSwitch::isActive()) {
                    mState = State::Off;
                }
                else {
                    mState = State::On;
                }
            });
            break;
        case State::Off:
            break;
        }
        if (oldState != mState) {
            mStateTicks.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::PreOn:
                devs::blinkLed2::blink(blink2_t{2});
                devs::powerOn::init();
                devs::powerOn::activate();
                break;
            case State::On:
                devs::blinkLed2::blink(blink2_t{2});
                break;
            case State::PreOff:
                devs::blinkLed1::off();
                devs::blinkLed2::off();
//                devs::powerOn::inactivate();
                break;
            case State::Off:
                devs::powerOn::inactivate();
                break;
            }
        }
    
    }     
private:    
    static inline State mState{State::Undefined};
    static inline External::Tick<typename devs::systemTimer> mStateTicks;
};

using devices = Devices<1>;
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
