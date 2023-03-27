#include "devices.h"

template<typename Devices>
struct GlobalFsm {
    using devs = Devices;
    
    enum class State : uint8_t {Undefined, On};

    static inline constexpr External::Tick<typename devs::systemTimer> powerTicks{1000_ms};
    static inline constexpr External::Tick<typename devs::systemTimer> debugTicks{500_ms};
    
    using blink1_t = devs::blinkLed1::count_type;
    using blink2_t = devs::blinkLed2::count_type;
    using terminal1 = etl::basic_ostream<typename devs::serial1>;
    using terminal2 = etl::basic_ostream<typename devs::serial2>;
    
    static void init() {
        devs::init();
        
        devs::blinkLed1::init();
        devs::blinkLed2::init();

        devs::serial1::template init<AVR::BaudRate<9600>>();
        devs::serial2::template init<AVR::BaudRate<9600>>();

        devs::hc05::init();
    } 
    static void periodic() {
        devs::la0::toggle();
        
        devs::serial1::periodic();
        devs::serial2::periodic();
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
                mState = State::On;
            });
            break;
        case State::On:
            mStateTicks.on(debugTicks, []{
                etl::outl<terminal1>("serial1: test05"_pgm);                
                etl::outl<terminal2>("serial2: test05"_pgm);                
            });
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
