#define USE_MCU_STM_V2
#define NDEBUG

#include "devices.h"

#include <chrono>
#include <cassert>

using namespace std::literals::chrono_literals;

struct Config {
    Config() = delete;
};

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;
    
    using pwm1 = devs::pwm1;
    using crsf = devs::crsf;
    
    using led = devs::led;
    
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    
    enum class State : uint8_t {Undefined, Init, Run};
    
    static inline void init() {
        devs::init();
        devs::led::set();
    }   
    static inline void periodic() {
        trace::periodic();
        crsf::periodic();
    }
    static inline void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(debugTicks, []{
                mState = State::Init;
            });
        break;
        case State::Init:
            mStateTick.on(debugTicks, []{
                mState = State::Run;
            });
        break;
        case State::Run:
            (++mDebugTick).on(debugTicks, []{
                led::toggle();
//                IO::outl<trace>("ch0: ", servo_pa::value(channel_t{0}), " p: ", servo_pa::packages(), " b: ", servo_pa::bytes(), " s: ", servo_pa::starts()); 
//                                IO::outl<trace>("req: ", sensor::ProtocollAdapter::requests(), "rpy: ", sensor::replies()); 
//                IO::outl<trace>("req: ", crsf::ProtocollAdapter::requests()); 
            });
        break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            
        }
    }

private:
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline State mState{State::Undefined};
};

void __assert_func (const char *, int, const char *, const char *){
    while(true) {
    }
}

int main() {
    using devs = Devices<CC01, Config, Mcu::Stm::Stm32G431>;
    using gfsm = GFSM<devs>;
    gfsm::init();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}
